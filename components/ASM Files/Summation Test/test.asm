# This program sums an array from memory. It stores the result in s0
# and it also saves the sum of the previous elements in memory at each 
# index. This is meant to test the six instructions required for project A.
.text
.globl main
main:
lw $8, 0($0) #get the size of the array
addi $9, $0, 4 #address of the first element in memory
add $16, $0, $0 #zero out the register where the sum is saved
j check # jump to the condition
loop:
lw $10, 0($9) # get an element of the array
add $16, $10, $16 # add the element to the sum
sw $16, 0($9) # store the sum back to memory
addi $9, $9, 4 # get the address of the next element
addi $8, $8, -1 # decrement the counter 
check:
beq $8, $0, exit # check if the counter is zero.
j loop #jump to the loop
exit:
