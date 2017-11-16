library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity register_file is
  port(CLK            : in  std_logic;
       rs_sel         : in  std_logic_vector(4 downto 0); -- first read address    
       rt_sel         : in  std_logic_vector(4 downto 0); -- second read address
       w_data         : in  std_logic_vector(31 downto 0); -- write data
       w_sel          : in  std_logic_vector(4 downto 0); -- write address
       w_en           : in  std_logic; -- write enable
       reset          : in  std_logic; -- resets all registers to 0
       rs_data        : out std_logic_vector(31 downto 0); -- first read data
       rt_data        : out std_logic_vector(31 downto 0)); -- second read data
 end register_file;

architecture mixed of register_file is 

type register_array is array (31 downto 0) of std_logic_vector(31 downto 0);

signal registers : register_array := (others => (others => '0'));

begin

rs_data <= registers(to_integer(unsigned(rs_sel)));
rt_data <= registers(to_integer(unsigned(rt_sel)));

write_process : process(CLK, reset)
	variable v_w_sel : integer := 0;
begin

	v_w_sel := to_integer(unsigned(w_sel));

	if(reset = '1') then
		registers <= (others => (others => '0'));
	elsif (falling_edge(CLK)) then
		if (w_en = '1' and w_sel /= "00000") then
			registers(v_w_sel) <= w_data; 
		end if;
	end if;

end process write_process;



end mixed;