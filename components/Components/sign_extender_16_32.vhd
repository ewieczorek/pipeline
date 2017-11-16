library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity sign_extender_16_32 is
  port(i_to_extend : in std_logic_vector(15 downto 0);
  	   o_extended : out std_logic_vector(31 downto 0));
 end sign_extender_16_32;

architecture mixed of sign_extender_16_32 is 

begin

with i_to_extend(15) select
	o_extended <= 	(x"ffff" & i_to_extend) when '1',
					(x"0000" & i_to_extend) when others;

end mixed;