#This is a program designed to test the functionality of your 
#Hazard Dection and Forwarding Unit. There are a series of 
#tests that do not depend upon one another so you may be able
#to complete some but not others.
.text
.globl main
main:
#FORWARDING UNIT
#Test 1: ALU Producer to ALU Consumer EX/MEM to EX
addi $t0, $zero, 1
add $t1, $zero, $t0
add $t2, $t1, $zero
nop
nop
#Test 2: ALU Producer to ALU Consumer MEM/WB to EX
addi $t0, $zero, 4 
addi $t1, $zero, -1
add $t2, $zero, $t0
add $t3, $t1, $zero
nop
nop
#Test 3: ALU Producer to ALU Consumer Precedence Test
#Forward from EX/MEM before MEM/WB
addi $t0, $zero, 6
addi $t0, $zero, 5
add $t1, $t0, $t0
nop
nop
#Test 4: Zero Register Forwarding
#Do not forward the Zero Register
addi $t0, $zero, 10
addi $zero, $zero, 8
add $t1, $zero, $zero
add $t2, $zero, $t0
nop
nop
#Test 5: Forwarding to a SW
addi $t0, $zero, 4
addi $t1, $zero -100
sw $t1, 0($t0)
nop
nop
#Test 6: Forwarding to a LW
addi $t0, $zero, 8
lw $t1, 0($t0)
addi $t0, $zero, 12
nop
lw $t2, 0($t0)
nop
nop
#Test 7: Forwarind to Branch
addi $t0, $zero, 0
nop
nop
nop
addi $t0, $zero, 35
nop
beq $t0, $zero, test7
nop
nop
nop
test7:
nop
nop
nop
#HAZARD DETECTION
#Test 8: Jump Followed by an Add
j test8
add $t1, $zero, 10
nop
nop
nop
test8:
nop
nop
#Test 9: Branch Taken then Add
beq $zero, $zero, test9
addi $t0, $zero, 12
nop
nop
nop
test9:
addi $t0, $zero, 20
nop
nop
#Test 10: Branch Not Taken then add
addi $t0, $zero, 11
nop
nop
nop
beq $t0, $zero, test10
add $t1, $t0, $t0
nop
nop
nop
test10:
nop
nop
nop
#Test 11: Add followed by a branch
addi $t0, $zero, 0
nop
nop
nop
addi $t0, $zero 40
beq $t0, $zero test11
addi $t1, $zero, 50
nop
nop
test11:
nop
nop
nop
#NEW TESTS
#Test 12: Lw to Add distance 1
add $t0, $zero, $zero 
nop
nop
nop
lw $t0, 20($zero)
add $t1, $t0, $t0
nop
nop
nop 
#Test 13: Lw to Add distance 2
add $t0, $zero, $zero
nop
nop
nop
lw $t0, 24($zero)
nop
add $t1, $t0, $t0
nop
nop
nop