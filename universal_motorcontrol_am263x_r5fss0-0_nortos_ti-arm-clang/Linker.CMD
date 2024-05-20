/* This is the stack that is used by code running within main()

* In case of NORTOS,

* - This means all the code outside of ISR uses this stack

* In case of FreeRTOS

* - This means all the code until vTaskStartScheduler() is called in main()

*   uses this stack.

* - After vTaskStartScheduler() each task created in FreeRTOS has its own stack

*/

--stack_size=16384

/* This is the heap size for malloc() API in NORTOS and FreeRTOS

* This is also the heap used by pvPortMalloc in FreeRTOS

*/

--heap_size=32768

-e_vectors  /* This is the entry of the application, _vector MUST be plabed starting address 0x0 */



/* This is the size of stack when R5 is in IRQ mode

* In NORTOS,

* - Here interrupt nesting is enabled

* - This is the stack used by ISRs registered as type IRQ

* In FreeRTOS,

* - Here interrupt nesting is enabled

* - This is stack that is used initally when a IRQ is received

* - But then the mode is switched to SVC mode and SVC stack is used for all user ISR callbacks

* - Hence in FreeRTOS, IRQ stack size is less and SVC stack size is more

*/

__IRQ_STACK_SIZE = 256;

/* This is the size of stack when R5 is in IRQ mode

* - In both NORTOS and FreeRTOS nesting is disabled for FIQ

*/

__FIQ_STACK_SIZE = 256;

__SVC_STACK_SIZE = 4096; /* This is the size of stack when R5 is in SVC mode */

__ABORT_STACK_SIZE = 256;  /* This is the size of stack when R5 is in ABORT mode */

__UNDEFINED_STACK_SIZE = 256;  /* This is the size of stack when R5 is in UNDEF mode */



SECTIONS

{

    /* This has the R5F entry point and vector table, this MUST be at 0x0 */

    .vectors:{} palign(8) > R5F_VECS





       /* This is to put CycleCounterP_getCount32 in TCM */

    GROUP {

        .text.pmu: palign(8)

    } > R5F_TCMA



    GROUP {

        .data.pmu: palign(8)

    } > R5F_TCMB



    GROUP {

        .text.hwi: palign(8)

    } > R5F_TCMB



    /* This has the R5F boot code until MPU is enabled,  this MUST be at a address < 0x80000000

     * i.e this cannot be placed in DDR

     */

    GROUP {

        .text.cache: palign(8)

        .text.mpu: palign(8)

        .text.boot: palign(8)

        .text:abort: palign(8) /* this helps in loading symbols when using XIP mode */

    } > OCRAM



    /* This is rest of code. This can be placed in DDR if DDR is available and needed */

    GROUP {

        .text:   {} palign(8)   /* This is where code resides */

        .rodata: {} palign(8)   /* This is where const's go */

    } > OCRAM



    /* This is rest of initialized data. This can be placed in DDR if DDR is available and needed */

    GROUP {



        .data:   {} palign(8)   /* This is where initialized globals and static go */

    } > OCRAM



    /* This is rest of uninitialized data. This can be placed in DDR if DDR is available and needed */

    GROUP {

        .bss:    {} palign(8)   /* This is where uninitialized globals go */

        RUN_START(__BSS_START)

        RUN_END(__BSS_END)

    } > R5F_TCMB



        GROUP {

        .sysmem: {} palign(8)   /* This is where the malloc heap goes */

        .stack:  {} palign(8)   /* This is where the main() stack goes */

    } > OCRAM



    /* This is where the stacks for different R5F modes go */

    GROUP {

        .irqstack: {. = . + __IRQ_STACK_SIZE;} align(8)

        RUN_START(__IRQ_STACK_START)

        RUN_END(__IRQ_STACK_END)

        .fiqstack: {. = . + __FIQ_STACK_SIZE;} align(8)

        RUN_START(__FIQ_STACK_START)

        RUN_END(__FIQ_STACK_END)

        .svcstack: {. = . + __SVC_STACK_SIZE;} align(8)

        RUN_START(__SVC_STACK_START)

        RUN_END(__SVC_STACK_END)

        .abortstack: {. = . + __ABORT_STACK_SIZE;} align(8)

        RUN_START(__ABORT_STACK_START)

        RUN_END(__ABORT_STACK_END)

        .undefinedstack: {. = . + __UNDEFINED_STACK_SIZE;} align(8)

        RUN_START(__UNDEFINED_STACK_START)

        RUN_END(__UNDEFINED_STACK_END)

    } > R5F_TCMB



    /* Sections needed for C++ projects */

    GROUP {

        .ARM.exidx:  {} palign(8)   /* Needed for C++ exception handling */

        .init_array: {} palign(8)   /* Contains function pointers called before main */

        .fini_array: {} palign(8)   /* Contains function pointers called after main */

    } > OCRAM





       // commented to increase OCRAM

    /* General purpose user shared memory, used in some examples */

    //.bss.user_shared_mem (NOLOAD) : {} > USER_SHM_MEM

    /* this is used when Debug log's to shared memory are enabled, else this is not used */

    //.bss.log_shared_mem  (NOLOAD) : {} > LOG_SHM_MEM





    /* this is used only when IPC RPMessage is enabled, else this is not used */

    .bss.ipc_vring_mem   (NOLOAD) : {} > RTOS_NORTOS_IPC_SHM_MEM







    /* this is used only when Secure IPC is enabled */

    /*

    .bss.sipc_hsm_queue_mem   (NOLOAD) : {} > MAILBOX_HSM

    .bss.sipc_r5f_queue_mem   (NOLOAD) : {} > MAILBOX_R5F

    */



    .tcm_code :      {}     > R5F_TCMA

    .tcm_data :      {}     > R5F_TCMB

    hal_data :      {}     > R5F_TCMB

    user_data :      {}     > R5F_TCMB

    foc_data :      {}     > R5F_TCMB

    motor_data      :      {}     > R5F_TCMB

    sys_data :      {}     > R5F_TCMB

    .sfradata :      {}     > R5F_TCMB

    datalog_data    :      {}     > OCRAM







    /*

    For debugging purposes

    .tcm_code :      {}     > R5F_TCMA

                          LOAD_START(loadStart_tcma_code),

                          LOAD_END(loadEnd_tcma_code),

                          LOAD_SIZE(loadSize_tcma_code)

    .tcm_data :      {}     > R5F_TCMB

                          LOAD_START(loadStart_tcma_data),

                          LOAD_END(loadEnd_tcma_data),

                          LOAD_SIZE(loadSize_tcma_data)

    hal_data :      {}     > OCRAM

                          LOAD_START(loadStart_hal_data),

                         LOAD_END(loadEnd_hal_data),

                          LOAD_SIZE(loadSize_hal_data)

    user_data :      {}     > OCRAM

                          LOAD_START(loadStart_user_data),

                          LOAD_END(loadEnd_user_data),

                          LOAD_SIZE(loadSize_user_data)

    foc_data :      {}     > OCRAM

                          LOAD_START(loadStart_foc_data),

                          LOAD_END(loadEnd_foc_data),

                          LOAD_SIZE(loadSize_foc_data)

    motor_data      :      {}     > OCRAM

                          LOAD_START(loadStart_motor_data),

                          LOAD_END(loadEnd_motor_data),

                          LOAD_SIZE(loadSize_motor_data)

    sys_data :      {}     > OCRAM

                          LOAD_START(loadStart_sys_data),

                          LOAD_END(loadEnd_sys_data),

                          LOAD_SIZE(loadSize_sys_data)

    datalog_data    :      {}     > OCRAM

                          LOAD_START(loadStart_datalog_data),

                          LOAD_END(loadEnd_datalog_data),

                          LOAD_SIZE(loadSize_datalog_data)

    .sfradata :      {}     > OCRAM

                          LOAD_START(loadStart_sfradata),

                          LOAD_END(loadEnd_sfradata),

                          LOAD_SIZE(loadSize_sfradata)

*/





}



MEMORY

{

    R5F_VECS  : ORIGIN = 0x00000000 , LENGTH = 0x00000040



    R5F_TCMA  : ORIGIN = 0x00000040 , LENGTH = 0x00007FC0

    R5F_TCMB  : ORIGIN = 0x00080000 , LENGTH = 0x00008000



    /* when using multi-core application's i.e more than one R5F/M4F active, make sure

     * this memory does not overlap with other R5F's

     */

    OCRAM     : ORIGIN = 0x70040000 , LENGTH = 0x200000



    /* This section can be used to put XIP section of the application in flash, make sure this does not overlap with

     * other CPUs. Also make sure to add a MPU entry for this section and mark it as cached and code executable

     */

    FLASH     : ORIGIN = 0x60100000 , LENGTH = 0x80000





    /* shared memories that are used by RTOS/NORTOS cores */

    /* On R5F,

     * - make sure there is a MPU entry which maps below regions as non-cache

     */



    /* To in crease OCRAM   */

    /*

    USER_SHM_MEM            : ORIGIN = 0x701D0000, LENGTH = 0x00004000

    LOG_SHM_MEM             : ORIGIN = 0x701D4000, LENGTH = 0x00004000

    */





    /* MSS mailbox memory is used as shared memory, we dont use bottom 32*12 bytes, since its used as SW queue by ipc_notify */

    RTOS_NORTOS_IPC_SHM_MEM : ORIGIN = 0x72000000, LENGTH = 0x3E80





    /*

    MAILBOX_HSM:    ORIGIN = 0x44000000 , LENGTH = 0x000003CE

    MAILBOX_R5F:    ORIGIN = 0x44000400 , LENGTH = 0x000003CE

    */





    }
