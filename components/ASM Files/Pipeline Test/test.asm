# This is the same program from Project A. It sums an array from memory. 
# It stores the result in s0 and it also saves the sum of the previous 
# elements in memory at each index. This is meant to test the pipeline
# without the need for forwarding and hazard detection.

.text
.globl main
main:
lw $8, 0($0) #get the size of the array
nop
nop
nop
nop
addi $9, $0, 4 #address of the first element in memory
nop
nop
nop
nop
add $16, $0, $0 #zero out the register where the sum is saved
nop
nop
nop
nop
j check # jump to the condition
nop
nop
nop
nop
loop:
nop
nop
nop
nop
lw $10, 0($9) # get an element of the array
nop
nop
nop
nop
add $16, $10, $16 # add the element to the sum
nop
nop
nop
nop
sw $16, 0($9) # store the sum back to memory
nop
nop
nop
nop
addi $9, $9, 4 # get the address of the next element
nop
nop
nop
nop
addi $8, $8, -1 # decrement the counter 
nop
nop
nop
nop
check:
nop
nop
nop
nop
beq $8, $0, exit # check if the counter is zero.
nop
nop
nop
nop
j loop #jump to the loop
nop
nop
nop
nop
exit:
