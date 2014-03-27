#include <assert.h>
#include "cpu.h"
#include "helper.h"
#include "qemu/host-utils.h"

#include "hw/lm32/lm32_pic.h"
#include "hw/char/lm32_juart.h"

#include "exec/softmmu_exec.h"

#ifndef CONFIG_USER_ONLY
#include "sysemu/sysemu.h"
#endif

#if !defined(CONFIG_USER_ONLY)
#define MMUSUFFIX _mmu
#define SHIFT 0
#include "exec/softmmu_template.h"
#define SHIFT 1
#include "exec/softmmu_template.h"
#define SHIFT 2
#include "exec/softmmu_template.h"
#define SHIFT 3
#include "exec/softmmu_template.h"

void raise_exception(CPULM32State *env, int index)
{
    env->exception_index = index;
    cpu_loop_exit(env);
}

void HELPER(raise_exception)(CPULM32State *env, uint32_t index)
{
    raise_exception(env, index);
}

void HELPER(hlt)(CPULM32State *env)
{
    CPUState *cs = CPU(lm32_env_get_cpu(env));

    cs->halted = 1;
    env->exception_index = EXCP_HLT;
    cpu_loop_exit(env);
}

void HELPER(ill)(CPULM32State *env)
{
#ifndef CONFIG_USER_ONLY
    CPUState *cs = CPU(lm32_env_get_cpu(env));
    fprintf(stderr, "VM paused due to illegal instruction. "
            "Connect a debugger or switch to the monitor console "
            "to find out more.\n");
    qemu_system_vmstop_request(RUN_STATE_PAUSED);
    cs->halted = 1;
    raise_exception(env, EXCP_HALTED);
#endif
}

void HELPER(wcsr_bp)(CPULM32State *env, uint32_t bp, uint32_t idx)
{
    uint32_t addr = bp & ~1;

    assert(idx < 4);

    env->bp[idx] = bp;
    lm32_breakpoint_remove(env, idx);
    if (bp & 1) {
        lm32_breakpoint_insert(env, idx, addr);
    }
}

void HELPER(wcsr_wp)(CPULM32State *env, uint32_t wp, uint32_t idx)
{
    lm32_wp_t wp_type;

    assert(idx < 4);

    env->wp[idx] = wp;

    wp_type = lm32_wp_type(env->dc, idx);
    lm32_watchpoint_remove(env, idx);
    if (wp_type != LM32_WP_DISABLED) {
        lm32_watchpoint_insert(env, idx, wp, wp_type);
    }
}

void HELPER(wcsr_dc)(CPULM32State *env, uint32_t dc)
{
    uint32_t old_dc;
    int i;
    lm32_wp_t old_type;
    lm32_wp_t new_type;

    old_dc = env->dc;
    env->dc = dc;

    for (i = 0; i < 4; i++) {
        old_type = lm32_wp_type(old_dc, i);
        new_type = lm32_wp_type(dc, i);

        if (old_type != new_type) {
            lm32_watchpoint_remove(env, i);
            if (new_type != LM32_WP_DISABLED) {
                lm32_watchpoint_insert(env, i, env->wp[i], new_type);
            }
        }
    }
}

void HELPER(wcsr_im)(CPULM32State *env, uint32_t im)
{
    lm32_pic_set_im(env->pic_state, im);
}

void HELPER(wcsr_ip)(CPULM32State *env, uint32_t im)
{
    lm32_pic_set_ip(env->pic_state, im);
}

void HELPER(wcsr_jtx)(CPULM32State *env, uint32_t jtx)
{
    lm32_juart_set_jtx(env->juart_state, jtx);
}

void HELPER(wcsr_jrx)(CPULM32State *env, uint32_t jrx)
{
    lm32_juart_set_jrx(env->juart_state, jrx);
}

uint32_t HELPER(rcsr_im)(CPULM32State *env)
{
    return lm32_pic_get_im(env->pic_state);
}

uint32_t HELPER(rcsr_ip)(CPULM32State *env)
{
    return lm32_pic_get_ip(env->pic_state);
}

uint32_t HELPER(rcsr_jtx)(CPULM32State *env)
{
    return lm32_juart_get_jtx(env->juart_state);
}

uint32_t HELPER(rcsr_jrx)(CPULM32State *env)
{
    return lm32_juart_get_jrx(env->juart_state);
}

void HELPER(wcsr_psw)(CPULM32State *env, uint32_t psw)
{

    env->psw = psw & PSW_MASK;
    if (psw & (PSW_ITLB | PSW_DTLB)) {
        tlb_flush(env, 1);
    }
}

void HELPER(wcsr_tlbvaddr)(CPULM32State *env, uint32_t tlbvaddr)
{
    env->tlbvaddr = tlbvaddr;

    switch (tlbvaddr & TLB_CMD_MASK) {
    case ITLB_NOP:
    case DTLB_NOP:
        break;
    case ITLB_INVALIDATE:
        mmu_invalidate_tlb(env, env->mmu.itlb);
        break;
    case DTLB_INVALIDATE:
        mmu_invalidate_tlb(env, env->mmu.dtlb);
        break;
    case ITLB_FLUSH:
        mmu_flush_tlb(env, env->mmu.itlb);
        break;
    case DTLB_FLUSH:
        mmu_flush_tlb(env, env->mmu.dtlb);
        break;
    }
}

void HELPER(wcsr_tlbpaddr)(CPULM32State *env, uint32_t tlbpaddr)
{
    if (tlbpaddr & 1) {
        mmu_fill_tbl(env, env->mmu.dtlb, tlbpaddr);
    } else {
        mmu_fill_tbl(env, env->mmu.itlb, tlbpaddr);
    }
}

/* Try to fill the TLB and return an exception if error. If retaddr is
   NULL, it means that the function was called in C code (i.e. not
   from generated code or from helper.c) */
void tlb_fill(CPULM32State *env, target_ulong addr, int is_write, int mmu_idx,
              uintptr_t retaddr)
{
    int ret;

    ret = cpu_lm32_handle_mmu_fault(env, addr, is_write, mmu_idx);
    if (unlikely(ret)) {
        if (retaddr) {
            /* now we have a real cpu fault */
            cpu_restore_state(env, retaddr);
        }
        cpu_loop_exit(env);
    }
}
#endif

