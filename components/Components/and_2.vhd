library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity and_2 is
  port( i_A, i_B : in std_logic;
  	    o_F : out std_logic);
 end and_2;

architecture mixed of and_2 is 

begin

o_F <= i_A and i_B;

end mixed;