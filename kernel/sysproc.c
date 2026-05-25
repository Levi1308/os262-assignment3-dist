#include "types.h"
#include "riscv.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "proc.h"

uint64
sys_exit(void)
{
  int n;
  argint(0, &n);
  exit(n);
  return 0;  // not reached
}

uint64
sys_getpid(void)
{
  return myproc()->pid;
}

uint64
sys_fork(void)
{
  return fork();
}

uint64
sys_wait(void)
{
  uint64 p;
  argaddr(0, &p);
  return wait(p);
}

uint64
sys_sbrk(void)
{
  uint64 addr;
  int n;

  argint(0, &n);
  addr = myproc()->sz;
  if(growproc(n) < 0)
    return -1;
  return addr;
}

uint64
sys_sleep(void)
{
  int n;
  uint ticks0;

  argint(0, &n);
  acquire(&tickslock);
  ticks0 = ticks;
  while(ticks - ticks0 < n){
    if(killed(myproc())){
      release(&tickslock);
      return -1;
    }
    sleep(&ticks, &tickslock);
  }
  release(&tickslock);
  return 0;
}

uint64
sys_kill(void)
{
  int pid;

  argint(0, &pid);
  return kill(pid);
}

// return how many clock tick interrupts have occurred
// since start.
uint64
sys_uptime(void)
{
  uint xticks;

  acquire(&tickslock);
  xticks = ticks;
  release(&tickslock);
  return xticks;
}

// sys_flip_display: zero-copy page flip.
//
// Syscall argument 0: user virtual address of a page-aligned buffer
// that is exactly GPU_FB_PAGES (300) * PGSIZE bytes (i.e. 640x480x4 =
// 1,228,800 bytes).  The buffer must already be fully mapped in the
// calling process's address space.
//
// TODO: Students implement this syscall.
uint64
sys_flip_display(void)
{
  uint64 buf;
  uint64 len = (uint64)GPU_FB_PAGES * PGSIZE;
  struct proc *p = myproc();
  uint64 *pa_list;
  int ret = -1;

  argaddr(0, &buf);

  if(buf == 0)
    return -1;
  if((buf % PGSIZE) != 0)
    return -1;
  if(buf + len < buf)
    return -1;
  if(buf >= MAXVA || buf + len > MAXVA)
    return -1;

  pa_list = (uint64 *)kalloc();
  if(pa_list == 0)
    return -1;

  for(int i = 0; i < GPU_FB_PAGES; i++){
    uint64 va = buf + (uint64)i * PGSIZE;
    pte_t *pte = walk(p->pagetable, va, 0);

    if(pte == 0)
      goto out;
    if((*pte & PTE_V) == 0)
      goto out;
    if((*pte & PTE_U) == 0)
      goto out;
    if(PTE_FLAGS(*pte) == PTE_V)
      goto out;

    pa_list[i] = PTE2PA(*pte);
    if(pa_list[i] == 0 || (pa_list[i] % PGSIZE) != 0)
      goto out;
  }

  if(virtio_gpu_flip(pa_list, GPU_FB_PAGES) < 0)
    goto out;

  ret = 0;

out:
  kfree((void *)pa_list);
  return ret;
}

// sys_map_display: map the GPU's kernel framebuffer pages (fb[]) directly
// into the calling process's address space with PTE_U|PTE_R|PTE_W.
//
// Syscall argument 0: desired user virtual address (must be page-aligned).
//   Pass 0 to let the kernel auto-select the next available VA above p->sz.
//
// Returns the mapped virtual address on success, (uint64)-1 on failure.
//
// TODO: Students implement this syscall.
uint64
sys_map_display(void)
{
  uint64 addr;
  uint64 base;
  uint64 len = (uint64)GPU_FB_PAGES * PGSIZE;
  struct proc *p = myproc();
  int mapped = 0;

  argaddr(0, &addr);

  if(addr == 0){
    base = PGROUNDUP(p->sz);
  } else {
    if(addr % PGSIZE)
      return -1;
    base = addr;
  }

  if(p->display_map_npages != 0)
    return -1;

  // Ensure the display scans out from kernel fb[] when using map mode.
  if(virtio_gpu_use_kernel_fb() < 0)
    return -1;

  // Reject invalid user ranges before touching page tables.
  if(base >= TRAPFRAME)
    return -1;
  if(base + len < base)
    return -1;
  if(base + len > TRAPFRAME)
    return -1;

  // Verify target VA range is entirely unmapped to avoid mappages() panic.
  for(int i = 0; i < GPU_FB_PAGES; i++){
    uint64 va = base + (uint64)i * PGSIZE;
    pte_t *pte = walk(p->pagetable, va, 0);
    uint64 pa;

    if(pte && (*pte & PTE_V))
      return -1;
    if(virtio_gpu_fb_page_pa(i, &pa) < 0)
      return -1;
    if((pa % PGSIZE) != 0)
      return -1;
  }

  // Map all GPU framebuffer pages with user RW permissions.
  for(int i = 0; i < GPU_FB_PAGES; i++){
    uint64 pa;
    uint64 va = base + (uint64)i * PGSIZE;

    if(virtio_gpu_fb_page_pa(i, &pa) < 0)
      goto rollback;
    if(mappages(p->pagetable, va, PGSIZE, pa, PTE_U|PTE_R|PTE_W) < 0)
      goto rollback;
    mapped++;
  }

  p->display_map_base = base;
  p->display_map_npages = GPU_FB_PAGES;

  return base;

rollback:
  if(mapped > 0)
    uvmunmap(p->pagetable, base, mapped, 0);
  return -1;
}
