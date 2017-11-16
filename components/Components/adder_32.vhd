library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity adder_32 is
  port( i_A, i_B : in std_logic_vector(31 downto 0);
  	    o_F : out std_logic_vector(31 downto 0));
 end adder_32;

architecture mixed of adder_32 is 

begin

o_F <= std_logic_vector(unsigned(i_A) + unsigned(i_B));

end mixed;