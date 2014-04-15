/*
 *  LatticeMico32 helper routines.
 *
 *  Copyright (c) 2010-2013 Michael Walle <michael@walle.cc>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "cpu.h"
#include "qemu/host-utils.h"

static int get_physical_address(LM32CPU *cpu,
        hwaddr *physical, int *prot, target_ulong address, int rw)
{
    CPULM32State *env = &cpu->env;
    int idx;
    uint8_t current_asid;
    bool dtlb = (rw != 2);
    bool tlb_enabled = (dtlb) ? env->psw & PSW_DTLB : env->psw & PSW_ITLB;
    tlb_t *tlb;

    *prot = 0;

    if (cpu->def->features & LM32_FEATURE_MMU && tlb_enabled) {
        idx = mmu_tlb_index(env, address);

        if (dtlb) {
            tlb = &env->mmu.dtlb[idx];
        } else {
            tlb = &env->mmu.itlb[idx];
        }

        if (!tlb->valid) {
            /* TLB entry not valid */
            goto tlb_miss;
        }

        if (((tlb->vaddr ^ address) & 0xffc00000) != 0) {
            /* TLB frame number mismatch */
            goto tlb_miss;
        }

        current_asid = (env->psw & PSW_ASID_MASK) >> PSW_ASID_SHIFT;
        if (tlb->asid != current_asid) {
           printf("[qemu] asid missmatch. current == %d, tlbe asid == %d\n", current_asid, tlb->asid);
           goto tlb_miss;
        }

        /* for all ITLB entries ro == 0 */
        if (tlb->ro && rw == 1) {
            goto tlb_fault;
        }

        *physical = tlb->paddr;
        if (dtlb) {
            if (tlb->ro) {
                *prot = PAGE_READ;
            } else {
                *prot = PAGE_READ | PAGE_WRITE;
            }
        } else {
            *prot = PAGE_EXEC /* | PAGE_READ */;
        }
    } else {
        /* mmu not available or disabled */
        *physical = address;
        *prot = PAGE_BITS;
    }

    return 0;

tlb_miss:
    return 1;
tlb_fault:
    return 2;
}

void mmu_fill_tbl(CPULM32State *env, tlb_t *tlb, uint32_t tlbpaddr)
{
    int idx = mmu_tlb_index(env, env->tlbvaddr);
    tlb[idx].vaddr = env->tlbvaddr & TARGET_PAGE_MASK;
    tlb[idx].paddr = tlbpaddr & TARGET_PAGE_MASK;
    tlb[idx].valid = 1;
    tlb[idx].ro = !!(tlbpaddr & 2);
    tlb[idx].asid = (env->tlbvaddr & TLB_ASID_MASK) >> TLB_ASID_SHIFT;
    tlb_flush_page(env, env->tlbvaddr);
}

void mmu_flush_tlb(CPULM32State *env, tlb_t *tlb)
{
    int idx;
    for (idx = 0; idx < LM32_TLB_ENTRIES; idx++) {
        tlb[idx].valid = 0;
    }
    tlb_flush(env, 1);
}

void mmu_invalidate_tlb(CPULM32State *env, tlb_t *tlb)
{
    int idx = mmu_tlb_index(env, env->tlbvaddr);
    tlb[idx].valid = 0;
    tlb_flush_page(env, tlb[idx].vaddr);
}

static void raise_mmu_exception(CPULM32State *env, target_ulong address, int rw, int fault)
{
    /* already preset TLBVADDR and set TLBBADVADDR to miss address */
    env->tlbvaddr = address & TARGET_PAGE_MASK;
    env->tlbbadvaddr = address;

    if (rw == 0 || ((rw == 1) && (fault == 0))) {
        /* preset lower bit, indicating that DTLB is selected on next write
         * to TLBPADDR */
        env->tlbvaddr |= 1;
        env->exception_index = EXCP_DTLB_MISS;
    } else if (rw == 1) {
        env->tlbvaddr |= 1;
        env->exception_index = EXCP_DTLB_FAULT;
    } else {
        env->exception_index = EXCP_ITLB_MISS;
    }
}

int cpu_lm32_handle_mmu_fault(CPULM32State *env, target_ulong address, int rw,
                              int mmu_idx)
{
    LM32CPU *cpu = lm32_env_get_cpu(env);
    hwaddr physical;
    int prot;
    int ret;

    ret = get_physical_address(cpu, &physical, &prot, address, rw);

    if (!ret) {
        if (env->flags & LM32_FLAG_IGNORE_MSB) {
            physical &= 0x7fffffff;
        }

        tlb_set_page(env, address & TARGET_PAGE_MASK,
                physical & TARGET_PAGE_MASK, prot, mmu_idx,
                TARGET_PAGE_SIZE);
        ret = 0;
    } else {
        raise_mmu_exception(env, address, rw, ret == 2);
        ret = 1;
    }

    return ret;
}

hwaddr lm32_cpu_get_phys_page_debug(CPUState *cs, vaddr addr)
{
    LM32CPU *cpu = LM32_CPU(cs);

    addr &= TARGET_PAGE_MASK;
    if (cpu->env.flags & LM32_FLAG_IGNORE_MSB) {
        return addr & 0x7fffffff;
    } else {
        return addr;
    }
}

void lm32_breakpoint_insert(CPULM32State *env, int idx, target_ulong address)
{
    cpu_breakpoint_insert(env, address, BP_CPU, &env->cpu_breakpoint[idx]);
}

void lm32_breakpoint_remove(CPULM32State *env, int idx)
{
    if (!env->cpu_breakpoint[idx]) {
        return;
    }

    cpu_breakpoint_remove_by_ref(env, env->cpu_breakpoint[idx]);
    env->cpu_breakpoint[idx] = NULL;
}

void lm32_watchpoint_insert(CPULM32State *env, int idx, target_ulong address,
                            lm32_wp_t wp_type)
{
    int flags = 0;

    switch (wp_type) {
    case LM32_WP_DISABLED:
        /* nothing to to */
        break;
    case LM32_WP_READ:
        flags = BP_CPU | BP_STOP_BEFORE_ACCESS | BP_MEM_READ;
        break;
    case LM32_WP_WRITE:
        flags = BP_CPU | BP_STOP_BEFORE_ACCESS | BP_MEM_WRITE;
        break;
    case LM32_WP_READ_WRITE:
        flags = BP_CPU | BP_STOP_BEFORE_ACCESS | BP_MEM_ACCESS;
        break;
    }

    if (flags != 0) {
        cpu_watchpoint_insert(env, address, 1, flags,
                &env->cpu_watchpoint[idx]);
    }
}

void lm32_watchpoint_remove(CPULM32State *env, int idx)
{
    if (!env->cpu_watchpoint[idx]) {
        return;
    }

    cpu_watchpoint_remove_by_ref(env, env->cpu_watchpoint[idx]);
    env->cpu_watchpoint[idx] = NULL;
}

static bool check_watchpoints(CPULM32State *env)
{
    LM32CPU *cpu = lm32_env_get_cpu(env);
    int i;

    for (i = 0; i < cpu->def->num_watchpoints; i++) {
        if (env->cpu_watchpoint[i] &&
                env->cpu_watchpoint[i]->flags & BP_WATCHPOINT_HIT) {
            return true;
        }
    }
    return false;
}

void lm32_debug_excp_handler(CPULM32State *env)
{
    CPUBreakpoint *bp;

    if (env->watchpoint_hit) {
        if (env->watchpoint_hit->flags & BP_CPU) {
            env->watchpoint_hit = NULL;
            if (check_watchpoints(env)) {
                raise_exception(env, EXCP_WATCHPOINT);
            } else {
                cpu_resume_from_signal(env, NULL);
            }
        }
    } else {
        QTAILQ_FOREACH(bp, &env->breakpoints, entry)
            if (bp->pc == env->pc) {
                if (bp->flags & BP_CPU) {
                    raise_exception(env, EXCP_BREAKPOINT);
                }
                break;
            }
    }
}

void lm32_cpu_do_interrupt(CPUState *cs)
{
    LM32CPU *cpu = LM32_CPU(cs);
    CPULM32State *env = &cpu->env;

    qemu_log_mask(CPU_LOG_INT,
            "exception at pc=%x type=%x\n", env->pc, env->exception_index);

    switch (env->exception_index) {
    case EXCP_INSN_BUS_ERROR:
    case EXCP_DATA_BUS_ERROR:
    case EXCP_DIVIDE_BY_ZERO:
    case EXCP_IRQ:
    case EXCP_SYSTEMCALL:
    case EXCP_DTLB_MISS:
    case EXCP_ITLB_MISS:
    case EXCP_DTLB_FAULT:
    case EXCP_PRIVILEGE:
        /* non-debug exceptions */
        env->regs[R_EA] = env->pc;
        env->ie |= (env->ie & IE_IE) ? IE_EIE : 0;
        env->ie &= ~IE_IE;
        if (cpu->def->features & LM32_FEATURE_MMU) {
            env->psw |= (env->psw & PSW_ITLB) ? PSW_EITLB : 0;
            env->psw |= (env->psw & PSW_DTLB) ? PSW_EDTLB : 0;
            env->psw |= (env->psw & PSW_USR) ? PSW_EUSR : 0;
            env->psw &= ~(PSW_ITLB | PSW_DTLB | PSW_USR);
        }
        if (env->dc & DC_RE) {
            env->pc = env->deba + (env->exception_index * 32);
        } else {
            env->pc = env->eba + (env->exception_index * 32);
        }
        log_cpu_state_mask(CPU_LOG_INT, cs, 0);
        break;
    case EXCP_BREAKPOINT:
    case EXCP_WATCHPOINT:
        /* debug exceptions */
        env->regs[R_BA] = env->pc;
        env->ie |= (env->ie & IE_IE) ? IE_BIE : 0;
        env->ie &= ~IE_IE;
        if (cpu->def->features & LM32_FEATURE_MMU) {
            env->psw |= (env->psw & PSW_ITLB) ? PSW_BITLB : 0;
            env->psw |= (env->psw & PSW_DTLB) ? PSW_BDTLB : 0;
            env->psw |= (env->psw & PSW_USR) ? PSW_BUSR : 0;
            env->psw &= ~(PSW_ITLB | PSW_DTLB | PSW_USR);
        }
        env->pc = env->deba + (env->exception_index * 32);
        log_cpu_state_mask(CPU_LOG_INT, cs, 0);
        break;
    default:
        cpu_abort(env, "unhandled exception type=%d\n",
                  env->exception_index);
        break;
    }
}

static const LM32Def lm32_defs[] = {
    {
        .name = "lm32-basic",
        .revision = 3,
        .num_interrupts = 32,
        .num_breakpoints = 4,
        .num_watchpoints = 4,
        .features = (LM32_FEATURE_SHIFT
                     | LM32_FEATURE_SIGN_EXTEND
                     | LM32_FEATURE_CYCLE_COUNT),
    },
    {
        .name = "lm32-standard",
        .revision = 3,
        .num_interrupts = 32,
        .num_breakpoints = 4,
        .num_watchpoints = 4,
        .features = (LM32_FEATURE_MULTIPLY
                     | LM32_FEATURE_DIVIDE
                     | LM32_FEATURE_SHIFT
                     | LM32_FEATURE_SIGN_EXTEND
                     | LM32_FEATURE_I_CACHE
                     | LM32_FEATURE_CYCLE_COUNT),
    },
    {
        .name = "lm32-full",
        .revision = 3,
        .num_interrupts = 32,
        .num_breakpoints = 4,
        .num_watchpoints = 4,
        .features = (LM32_FEATURE_MULTIPLY
                     | LM32_FEATURE_DIVIDE
                     | LM32_FEATURE_SHIFT
                     | LM32_FEATURE_SIGN_EXTEND
                     | LM32_FEATURE_I_CACHE
                     | LM32_FEATURE_D_CACHE
                     | LM32_FEATURE_CYCLE_COUNT),
    },
    {
        .name = "lm32-full-mmu",
        .revision = 3,
        .num_interrupts = 32,
        .num_breakpoints = 4,
        .num_watchpoints = 4,
        .features = (LM32_FEATURE_MULTIPLY
                     | LM32_FEATURE_DIVIDE
                     | LM32_FEATURE_SHIFT
                     | LM32_FEATURE_SIGN_EXTEND
                     | LM32_FEATURE_I_CACHE
                     | LM32_FEATURE_D_CACHE
                     | LM32_FEATURE_CYCLE_COUNT
                     | LM32_FEATURE_MMU),
    },
};

void cpu_lm32_list(FILE *f, fprintf_function cpu_fprintf)
{
    int i;

    cpu_fprintf(f, "Available CPUs:\n");
    for (i = 0; i < ARRAY_SIZE(lm32_defs); i++) {
        cpu_fprintf(f, "  %s\n", lm32_defs[i].name);
    }
}

static const LM32Def *cpu_lm32_find_by_name(const char *name)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(lm32_defs); i++) {
        if (strcasecmp(name, lm32_defs[i].name) == 0) {
            return &lm32_defs[i];
        }
    }

    return NULL;
}

static uint32_t cfg_by_def(const LM32Def *def)
{
    uint32_t cfg = 0;

    if (def->features & LM32_FEATURE_MULTIPLY) {
        cfg |= CFG_M;
    }

    if (def->features & LM32_FEATURE_DIVIDE) {
        cfg |= CFG_D;
    }

    if (def->features & LM32_FEATURE_SHIFT) {
        cfg |= CFG_S;
    }

    if (def->features & LM32_FEATURE_SIGN_EXTEND) {
        cfg |= CFG_X;
    }

    if (def->features & LM32_FEATURE_I_CACHE) {
        cfg |= CFG_IC;
    }

    if (def->features & LM32_FEATURE_D_CACHE) {
        cfg |= CFG_DC;
    }

    if (def->features & LM32_FEATURE_CYCLE_COUNT) {
        cfg |= CFG_CC;
    }

#if 0
    /* TODO */
    if (def->features & LM32_FEATURE_MMU) {
        cfg |= CFG_MMU;
    }
#endif

    cfg |= (def->num_interrupts << CFG_INT_SHIFT);
    cfg |= (def->num_breakpoints << CFG_BP_SHIFT);
    cfg |= (def->num_watchpoints << CFG_WP_SHIFT);
    cfg |= (def->revision << CFG_REV_SHIFT);

    return cfg;
}

LM32CPU *cpu_lm32_init(const char *cpu_model)
{
    LM32CPU *cpu;
    CPULM32State *env;
    const LM32Def *def;

    def = cpu_lm32_find_by_name(cpu_model);
    if (!def) {
        return NULL;
    }

    cpu = LM32_CPU(object_new(TYPE_LM32_CPU));
    cpu->def = def;

    env = &cpu->env;
    env->cfg = cfg_by_def(def);

    object_property_set_bool(OBJECT(cpu), true, "realized", NULL);

    return cpu;
}

/* Some soc ignores the MSB on the address bus. Thus creating a shadow memory
 * area. As a general rule, 0x00000000-0x7fffffff is cached, whereas
 * 0x80000000-0xffffffff is not cached and used to access IO devices. */
void cpu_lm32_set_phys_msb_ignore(CPULM32State *env, int value)
{
    if (value) {
        env->flags |= LM32_FLAG_IGNORE_MSB;
    } else {
        env->flags &= ~LM32_FLAG_IGNORE_MSB;
    }
}

static void dump_tlb(FILE *f, fprintf_function cpu_fprintf, CPULM32State *env,
        tlb_t *tlb)
{
    int idx;

    cpu_fprintf(f, "\tIdx     Vaddr       Paddr\n");
    cpu_fprintf(f, "\t------  ----------  ----------\n");
    for (idx = 0; idx < LM32_TLB_ENTRIES; idx++) {
        tlb_t *entry = &tlb[idx];
        if (entry->valid) {
            cpu_fprintf(f, "\t0x%04x  0x%08x  0x%08x\n", idx, entry->vaddr,
                    entry->paddr);
        }
    }
}
void dump_mmu(FILE *f, fprintf_function cpu_fprintf, CPULM32State *env)
{
    cpu_fprintf(f, "ITLB enabled: %s  DTLB enabled: %s\n",
            env->psw & PSW_ITLB ? "yes" : "no",
            env->psw & PSW_DTLB ? "yes" : "no");
    cpu_fprintf(f, "ITLB:\n");
    dump_tlb(f, cpu_fprintf, env, env->mmu.itlb);
    cpu_fprintf(f, "\nDTLB:\n");
    dump_tlb(f, cpu_fprintf, env, env->mmu.dtlb);
}
