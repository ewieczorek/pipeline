library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

-- shifts the input "i_to_shift" by 2 and output the result in "o_shifted"
entity sll_2 is
  port( i_to_shift : in std_logic_vector(31 downto 0);
  	    o_shifted : out std_logic_vector(31 downto 0));
 end sll_2;

architecture mixed of sll_2 is 

begin

o_shifted(31 downto 2) <= i_to_shift(29 downto 0);
o_shifted(1 downto 0) <= "00";

end mixed;