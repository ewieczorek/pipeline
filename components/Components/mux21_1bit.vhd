library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity mux21_1bit is
  port( i_0, i_1 : in std_logic;
  		i_sel : in std_logic;
  	    o_mux : out std_logic);
 end mux21_1bit;

architecture mixed of mux21_1bit is 

begin

with i_sel select
	o_mux <= 	i_1 when '1',
				i_0 when others;

end mixed;