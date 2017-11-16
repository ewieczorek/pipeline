# This program sums an array from memory. It stores the result in s0
# and it also saves the sum of the previous elements in memory at each 
# index. This is meant to test the six instructions required for project A.
.text
.globl main
main:
la $t5, n #load the address of n
lw $t0, 0($t5) #get n from memory
la $t1, list #get the address of the array in memory
add $s0, $zero, $zero #zero out the sum
j check #jump to the condition
loop:
lw $t2, 0($t1) #load the next element from the array
add $s0, $t2, $s0 #add to sum
sw $s0, 0($t1) #store the sum into the same address in memory
addi $t1, $t1, 4 #increment the pointer to the next element in the array
addi $t0, $t0, -1 # decrement the counter
check:
beq $t0, $zero, exit #check if the counter is zero
j loop #jump back to the loop
exit:

.data
n: .word 10
list: .word 1,2,3,4,5,6,7,8,9,10