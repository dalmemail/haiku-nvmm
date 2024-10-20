 Hardware virtualization for Haiku’s QEMU port 
=======================

This Google Summer of Code project aimed to bring hardware virtualization to Haiku by porting NVMM, a hypervisor that already has QEMU support, into Haiku from DragonFlyBSD. This would make using Haiku as their primary operating system a viable approach for more people.

This branch showcases the project status at the end of Google Summer of Code 2024 ([final report](https://www.haiku-os.org/blog/dalme/2024-08-19_gsoc_2024_hardware_virtualization_final_report/)). For further development see the hrev58265 branch.

Goals
------------
 * NVMM driver ported to Haiku (VMX backend only).
 * QEMU capable of accelerating virtual machines through NVMM.

Links
---------------
  * [Google Summer of Code proposal](https://github.com/dalmemail/haiku-nvmm/blob/master/proposal.pdf)
  * [Project thread on Haiku forums](https://discuss.haiku-os.org/t/gsoc-2024-hardware-acceleration-for-haikus-qemu-port/14784)
  * [Project page at GSoC archive](https://summerofcode.withgoogle.com/programs/2024/projects/7iuNzLBk)

Blog posts:
  * [[GSoC 2024] Hardware virtualization for Haiku’s QEMU port](https://www.haiku-os.org/blog/dalme/2024-05-11_gsoc_2024_hardware_virtualization_for_haikus_qemu_port/)
  * [[GSoC 2024] Hardware virtualization: Progress Report #1](https://www.haiku-os.org/blog/dalme/2024-06-27_gsoc_2024_hardware_virtualization_progress_report_1/)
  * [[GSoC 2024] Hardware Virtualization: Progress Report #2](https://www.haiku-os.org/blog/dalme/2024-07-24_gsoc_2024_hardware_virtualization_progress_report_2/)
  * [[GSoC 2024] Hardware Virtualization: Final Report](https://www.haiku-os.org/blog/dalme/2024-08-19_gsoc_2024_hardware_virtualization_final_report/)

![QEMU accelerating DOOM on KolibriOS through NVMM](https://github.com/dalmemail/haiku-nvmm/blob/master/kolibri-doom.png?raw=true)
