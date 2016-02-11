        .THUMB

        .ALIGN  2

        .GLOBL  DrawPixel16

        .THUMB_FUNC



output_grb:

        push    {r4, r5, lr}



        mov     r5, #240

        lsl     r5, r5,#1           @ r5 = 480

        mul     r5,r5,r1 @r3

        mov     r4,r0

        add     r5,r5,r4

        add     r5,r5,r4

        mov     r3,#192

        lsl     r3, r3, #19         @ r3 = 0x6000000

        add     r3,r3,r5

        mov     r4,r2

        strh    r4,[r3]



        pop     {r4, r5}

        pop     {r0}

        bx      r0

.fend1:
