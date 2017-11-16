library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity branch_comparator is
  port( i_rs_data, i_rt_data : in std_logic_vector(31 downto 0);
  	    o_equal : out std_logic); -- '1' if A==B, '0' otherwise
 end branch_comparator;

architecture mixed of branch_comparator is 

begin

compare: process(i_rs_data, i_rt_data)
begin

o_equal <= '0';

if(i_rs_data = i_rt_data) then
	o_equal <= '1';
end if;

end process compare;

end mixed;