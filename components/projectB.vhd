-- Author: Ethan Wieczorek
library IEEE;
use IEEE.std_logic_1164.all;
use work.all;

entity pipelinedProcessor is
	port (	reset : in std_logic;
		i_clock : in std_logic
		);
end pipelinedProcessor;

architecture dataflow of pipelinedProcessor is
signal byteEn : std_logic_vector(3 downto 0) := (others => '1'); --byteena is to be set to all 1s for this project
signal imemData : std_logic_vector(31 downto 0) := (others => '0');

--Below is all of the entity output signals going from left to right on the diagram
	--The inputs of all entities match up to output signals of other entities,
	--the corresponding output signal is directly wired to the inputs to avoid data loss and to avoid unecessary signals.
--First multiplexer on the far left, outputs to the pc register
signal pcMUX_out : std_logic_vector(31 downto 0);
--PC
signal PC_reg_out : std_logic_vector(31 downto 0);
--Instruction memory (IMEM)
signal IF_IMEM_out : std_logic_vector(31 downto 0);
signal ID_IMEM_out : std_logic_vector(31 downto 0);
signal EX_IMEM_out : std_logic_vector(31 downto 0);
signal MEM_IMEM_out : std_logic_vector(31 downto 0);
signal WB_IMEM_out : std_logic_vector(31 downto 0);
--32 bit adder top left
signal IF_adder1_out : std_logic_vector(31 downto 0);
signal ID_adder1_out : std_logic_vector(31 downto 0);
signal EX_adder1_out : std_logic_vector(31 downto 0);
signal MEM_adder1_out : std_logic_vector(31 downto 0);
signal WB_adder1_out : std_logic_vector(31 downto 0);
--Shift left outputs into the adder in EX stage.
signal SLL1_out : std_logic_vector(31 downto 0);
signal shiftInputConcat : std_logic_vector(31 downto 0) := (others => '0');
--Control unit outputs
signal ID_RegDst_out : std_logic;
signal Jump_out : std_logic;
signal ID_Branch_out : std_logic;
signal ID_MemtoReg_out : std_logic;
signal ID_ALUop_out : std_logic_vector(3 downto 0);
signal ID_MemWrite_out : std_logic;
signal ID_ALUsrc_out : std_logic;
signal ID_RegWrite_out : std_logic;
signal EX_RegDst_out : std_logic;
signal EX_Branch_out : std_logic;
signal EX_MemtoReg_out : std_logic;
signal EX_ALUop_out : std_logic_vector(3 downto 0);
signal EX_MemWrite_out : std_logic;
signal EX_ALUsrc_out : std_logic;
signal EX_RegWrite_out : std_logic;
signal MEM_RegDst_out : std_logic;
signal MEM_MemtoReg_out : std_logic;
signal MEM_RegWrite_out : std_logic;
signal MEM_MemWrite_out : std_logic;
signal WB_RegDst_out : std_logic;
signal WB_MemtoReg_out : std_logic;
signal WB_RegWrite_out : std_logic;
--First 5-bit mux, input to the register file
signal muxRegInput_out : std_logic_vector(4 downto 0);
--Register file
signal ID_RSdata : std_logic_vector(31 downto 0);
signal ID_RTdata : std_logic_vector(31 downto 0);
signal EX_RSdata : std_logic_vector(31 downto 0);
signal EX_RTdata : std_logic_vector(31 downto 0);
signal MEM_RTdata : std_logic_vector(31 downto 0);
--Sign extender 
signal ID_signExtend_out : std_logic_vector(31 downto 0);
signal EX_signExtend_out : std_logic_vector(31 downto 0);
--Second multiplexer, input to the ALU. 32 bit
signal muxALUInput_out : std_logic_vector(31 downto 0);
--Second adder, adding sign extended instruction and the output of the other adder. top right
signal EX_adder2_out : std_logic_vector(31 downto 0);
signal MEM_adder2_out : std_logic_vector(31 downto 0);
--ALU unit
signal zero_out : std_logic;
signal EX_ALU_out : std_logic_vector(31 downto 0);
signal MEM_ALU_out : std_logic_vector(31 downto 0);
signal WB_ALU_out : std_logic_vector(31 downto 0);
--And unit 
signal AND_out : std_logic;
--Multiplexer top right second adder output
signal muxAdderOutput_out : std_logic_vector(31 downto 0);
--Multiplexer top right output of the other multiplexer
signal muxMUXoutput_out : std_logic_vector(31 downto 0);
--signal mux1ValueInput : std_logic_vector(31 downto 0);
--Data memory outputs
signal MEM_dmemData_out : std_logic_vector(31 downto 0);
signal WB_dmemData_out : std_logic_vector(31 downto 0);
--bottom right multiplexer. output of the dmem and output of the ALU
signal muxDMEMoutput_out : std_logic_vector(31 downto 0);
--Multiplexer for Register Destination
--IF/ID

--ID/EX
signal EX_RSsel : in  std_logic_vector(4 downto 0);
signal EX_RTsel : in  std_logic_vector(4 downto 0);
signal EX_RDsel : in  std_logic_vector(4 downto 0);
--MEM/WB

--EX/MEM


begin
--IF STAGE
	--First Multiplexer outputs into the pc register
	--TODO Replace pcMUX_out
	MUX32BIT1 : entity work.mux21_32bit port map (i_0, i_1, i_sel, pcMUX_out);
	--PC register
	PCREG1 : entity work.PC_reg port map (i_clock, reset, pcMUX_out, PC_reg_out);
	--Instruction memory unit
	IMEM1 : entity work.imem port map (PC_reg_out(11 downto 2), byteEn, i_clock, imemData, '0', IF_IMEM_out);
	--32 bit adder top left of diagram
	ADD1 : entity work.adder_32 port map (PC_reg_out, x"00000004", IF_adder1_out);
--ID STAGE
	--Control Unit
	MC1 : entity work.main_control port map (ID_IMEM_out, ID_RegDst_out, Jump_out, ID_Branch_out, ID_MemtoReg_out, ID_ALUop_out, ID_MemWrite_out, ID_ALUsrc_out, ID_RegWrite_out);
	--Register file
	--TODO
	REG1 : entity work.register_file port map (i_clock, ID_IMEM_out(25 downto 21), ID_IMEM_out(20 downto 16), muxDMEMoutput_out, MemToReg_output, RegWrite_out, reset, ID_RSdata, ID_RTdata);
	--Sign extender
	SE1 : entity work.sign_extender_16_32 port map (ID_IMEM_out(15 downto 0), ID_signExtend_out);
	--Shift left load input to the 2nd adder, output of the sign extender. top right. 
--EX STAGE
	--Shift register, inputs into the adder in EX stage.
	SLL1 : entity work.sll_2 port map (EX_signExtend_out, SLL1_out);
	--Second adder, adding sign extended instruction and the output of the other adder. top right
	ADD2 : entity work.adder_32 port map (EX_adder1_out, SLL1_out, EX_adder2_out);
	--multiplexer, input to the ALU. 32 bit. EX stage.
	MUX32BIT1 : entity work.mux21_32bit port map (EX_RTdata, EX_signExtend_out, EX_ALUsrc_out, muxALUInput_out);
	--ALU unit
	ALU1 : entity work.ALU port map (EX_ALUop_out, x"00", EX_RSdata, muxALUInput_out, zero_out, EX_ALU_out);
--MEM STAGE
	--Multiplexer top right, second adder output, 32 bit
	MUX32BIT2 : entity work.mux21_32bit port map (ID_adder1_out, adder2_out, AND_out, muxAdderOutput_out);
	--Multiplexer top right output of the other multiplexer, 32 bit
	MUX32BIT3 : entity work.mux21_32bit port map (muxAdderOutput_out, SLL1_out, Jump_out, muxMUXoutput_out);
	--Data memory unit
	DMEM1 : entity work.dmem port map (ALU_out(11 downto 2), byteEn, i_clock, RTdata, MemWrite_out, dmemData_out);
--WRITE BACK STAGE
	--bottom right multiplexer. output of the dmem and output of the ALU, 32 bit.
	MUX32BIT4 : entity work.mux21_32bit port map (ALU_out, dmemData_out, MemtoReg_out, muxDMEMoutput_out);
	--MEMToReg mux. Inputs to register file.
	MUX5BIT1 : entity work.mux21_32bit port map (ID_IMEM_out(20 downto 16), ID_IMEM_out(15 downto 11), RegDst_out, muxRegInput_out);
	
	--PIPELINE REGISTERS
	--IF/ID unit
	IFID1 : entity work.if_id port map (i_clock, '0', '0', '0', IF_IMEM_out, ID_IMEM_out, IF_adder1_out, ID_adder1_out);
	--ID/EX unit
	IDEX1 : work.id_ex port map (
          CLK => i_clock,
  		ex_flush => '0',
		ex_stall => '0',
		idex_reset => '0',
		id_instruction => ID_IMEM_out,
       	ex_instruction  => EX_IMEM_out,
       	id_pc_plus_4 => ID_adder1_out,
       	ex_pc_plus_4 => EX_adder1_out,

        id_reg_dest => ID_RegDst_out,
  	    id_branch => ID_Branch_out,
  	    id_mem_to_reg => ID_MemtoReg_out,
  	    id_ALU_op => ID_ALUop_out,
  	    id_mem_write => ID_MemWrite_out,
  	    id_ALU_src => ID_ALUsrc_out,
  	    id_reg_write => ID_RegWrite_out,
  	    ex_reg_dest => EX_RegDst_out,
  	    ex_branch => EX_Branch_out,
  	    ex_mem_to_reg => EX_MemtoReg_out,
  	    ex_ALU_op => EX_ALUop_out,
  	    ex_mem_write => EX_MemWrite_out,
  	    ex_ALU_src => EX_ALUsrc_out,
  	    ex_reg_write => EX_RegWrite_out,

  		id_rs_data => ID_RSdata,
  		id_rt_data => ID_RTdata,
  		ex_rs_data => EX_RSdata,
  		ex_rt_data => EX_RTdata,
  		id_rs_sel => ID_IMEM_out(25 downto 21),
  		id_rt_sel => ID_IMEM_out(20 downto 16),
  		id_rd_sel => ID_IMEM_out(15 downto 11),
  		ex_rs_sel => EX_RSsel,
  		ex_rt_sel => EX_RTsel,
  		ex_rd_sel => EX_RDsel,

  		id_extended_immediate => ID_signExtend_out,
  		ex_extended_immediate = EX_signExtend_out);
		
	EXMEM1 : work.ex_mem port map (
		clk => i_clock,
		mem_flush => '0',
		mem_stall => '0',
		exmem_reset => '0',
		ex_instruction => EX_IMEM_out,
        mem_instruction => MEM_IMEM_out,
        ex_pc_plus_4 => EX_adder2_out,
       	mem_pc_plus_4 => MEM_adder2_out,

        ex_reg_dest => EX_RegDst_out,
  	    ex_mem_to_reg => EX_MemtoReg_out,
  	    ex_mem_write => EX_MemWrite_out,
  	    ex_reg_write => EX_RegWrite_out,
  	    mem_reg_dest => MEM_RegDst_out,
  	    mem_mem_to_reg => MEM_MemtoReg_out,
  	    mem_mem_write => MEM_MemWrite_out,
  	    mem_reg_write => MEM_RegWrite_out,

		ex_ALU_out => EX_ALU_out,
		mem_ALU_out => MEM_ALU_out,

		ex_rt_data => EX_RTdata,
		mem_rt_data => MEM_RTdata,
  		ex_write_reg_sel => , -- see the Reg. Dest. mux in the pipeline archteicture diagram
  		mem_write_reg_sel => );
		
	MEMWB1 : work.mem_wb port map (
		CLK => i_clock,
		wb_flush => '0',
		wb_stall => '0',
		memwb_reset => '0',
		mem_instruction => MEM_IMEM_out,
        wb_instruction => WB_IMEM_out,
        mem_pc_plus_4 => MEM_adder1_out,
       	wb_pc_plus_4 => WB_adder1_out,
		
        mem_reg_dest => MEM_RegDst_out,
  	    mem_mem_to_reg => MEM_MemtoReg_out,
  	    mem_reg_write => MEM_RegWrite_out,
  	    wb_reg_dest => WB_RegDst_out,
  	    wb_mem_to_reg => WB_MemtoReg_out,
  	    wb_reg_write => WB_RegWrite_out,

		mem_ALU_out => MEM_ALU_out,
		wb_ALU_out => WB_ALU_out,

		mem_dmem_out => MEM_dmemData_out,
		wb_dmem_out => WB_dmemData_out,

  		mem_write_reg_sel => ,
  		wb_write_reg_sel => );
end dataflow;

--The following comments show the information behind each of the entities needed for this projet.
--The first line of each section of information show the line used to call it as an entity.
--The lines following the entity initialization line show the different ports in more detail,
--this detail includes whether each port is a bus or a wire, and the size of it if it is a bus.
--The size information is used to make sure each port is matched to a correctly sized signal.

--32 bit adder information
--ADD1 : entity work.adder_32 port map (i_A, i_B, o_F);
--  port( i_A, i_B : in std_logic_vector(31 downto 0);
--  	    o_F : out std_logic_vector(31 downto 0));

--ALU information
--ALU1 : entity work.ALU port map (ALU_OP, shamt, i_A, i_B, zero, ALU_out);
--  port(ALU_OP        : in  std_logic_vector(3 downto 0);
--       shamt         : in  std_logic_vector(4 downto 0);
--       i_A           : in  std_logic_vector(31 downto 0);
--       i_B           : in  std_logic_vector(31 downto 0);
--       zero          : out std_logic;
--       ALU_out       : out std_logic_vector(31 downto 0));
	
--AND 2 information
--AND1 : entity work.and_2 port map (i_a, i_B, o_F);
--  port( i_A, i_B : in std_logic;
--  	    o_F : out std_logic);	

--Data memory (DMEM) information
--DMEM1 : entity work.dmem port map (address, byteena, clock, data, wren, q);
--	port   (address			: IN STD_LOGIC_VECTOR (depth_exp_of_2-1 DOWNTO 0) := (OTHERS => '0');
--			byteena			: IN STD_LOGIC_VECTOR (3 DOWNTO 0) := (OTHERS => '1');
--			clock			: IN STD_LOGIC := '1';
--			data			: IN STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
--			wren			: IN STD_LOGIC := '0';
--			q				: OUT STD_LOGIC_VECTOR (31 DOWNTO 0));    

--Instruction memory (IMEM) information
--IMEM1 : entity work.imem port map (address, byteena, clock, data, wren, q);
--	port   (address			: IN STD_LOGIC_VECTOR (depth_exp_of_2-1 DOWNTO 0) := (OTHERS => '0');
--			byteena			: IN STD_LOGIC_VECTOR (3 DOWNTO 0) := (OTHERS => '1');
--			clock			: IN STD_LOGIC := '1';
--			data			: IN STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
--			wren			: IN STD_LOGIC := '0';
--			q				: OUT STD_LOGIC_VECTOR (31 DOWNTO 0));  

--Main Control unit information
--MC1 : entity work.main_control port map (i_instruction, o_reg_dest, o_jump, o_branch, o_mem_to_reg, o_ALU_op, o_mem_write, o_ALU_src, o_reg_write);
--  port( i_instruction : in std_logic_vector(31 downto 0);
--  	    o_reg_dest : out std_logic;
--  	    o_jump : out std_logic;
--  	    o_branch : out std_logic;
--  	    o_mem_to_reg : out std_logic;
--  	    o_ALU_op : out std_logic_vector(3 downto 0);
--  	    o_mem_write : out std_logic;
--  	    o_ALU_src : out std_logic;
--  	    o_reg_write : out std_logic);

--2 to 1 multiplexer 1 bit information
--MUX1BIT1 : entity work.mux21_1bit port map (i_0, i_1, i_sel, o_mux);
--  port( i_0, i_1 : in std_logic;
--  		i_sel : in std_logic;
--  	    o_mux : out std_logic);

--2 to 1 multiplexer 5 bit information
--MUX5BIT1 : entity work.mux21_5bit port map (i_0, i_1, i_sel, o_mux);
--  port( i_0, i_1 : in std_logic_vector(4 downto 0);
--  		i_sel : in std_logic;
--  	    o_mux : out std_logic_vector(4 downto 0));

--2 to 1 multiplexer 32 bit information
--MUX32BIT1 : entity work.mux21_32bit port map (i_0, i_1, i_sel, o_mux);
--  port( i_0, i_1 : in std_logic_vector(31 downto 0);
--  		i_sel : in std_logic;
--  	    o_mux : out std_logic_vector(31 downto 0));

--PC register information
--PCREG1 : entity work.PC_reg port map (CLK, reset, i_next_PC, o_PC);
--  port(	CLK : in std_logic;
--  		reset : in std_logic;
--  		i_next_PC : in std_logic_vector(31 downto 0);
--  	   	o_PC : out std_logic_vector(31 downto 0));

--Register file information
--REG1 : entity work.register_file port map (CLK, rs_sel, rt_sel, w_data, w_sel, w_en, reset, rs_data, rt_data);
--  port(CLK            : in  std_logic;
--       rs_sel         : in  std_logic_vector(4 downto 0); -- first read address    
--       rt_sel         : in  std_logic_vector(4 downto 0); -- second read address
--       w_data         : in  std_logic_vector(31 downto 0); -- write data
--       w_sel          : in  std_logic_vector(4 downto 0); -- write address
--       w_en           : in  std_logic; -- write enable
--       reset          : in  std_logic; -- resets all registers to 0
--       rs_data        : out std_logic_vector(31 downto 0); -- first read data
--       rt_data        : out std_logic_vector(31 downto 0)); -- second read data

--Sign extender information
--SE1 : entity work.sign_extender_16_32 port map (i_to_extend, o_extended);
--  port(i_to_extend : in std_logic_vector(15 downto 0);
--  	   o_extended : out std_logic_vector(31 downto 0));

--Shift Left load information
--SLL1 : entity work.sll_2 port map (i_to_shift, o_shifted);
--  port( i_to_shift : in std_logic_vector(31 downto 0);
--  	    o_shifted : out std_logic_vector(31 downto 0));

--IF/ID port map.
--IFID1 : entity work.if_id port map (CLK, id_flush, id_stall, ifid_reset, if_instruction, id_instruction, if_pc_plus_4, id_pc_plus_4)
--  port(CLK            : in  std_logic;
--  		id_flush, id_stall, ifid_reset : in std_logic;
--       	if_instruction  : in std_logic_vector(31 downto 0);
--       	id_instruction  : out std_logic_vector(31 downto 0);
--       	if_pc_plus_4 : in std_logic_vector(31 downto 0);
--       	id_pc_plus_4 : out std_logic_vector(31 downto 0));

--ID/EX port map, this is a laid out style port map because there is too many ports to have on one line. 
--DO NOT USE IDEX1 : entity work.id_ex  port map (clk, ex_flush, ex_stall, idex_reset, id_instruction, ex_instruction, id_pc_plus_4, ex_pc_plus_4)
--IDEX1 : work.id_ex port map (
--          CLK => i_clock,
--  		ex_flush => ,
--			ex_stall => ,
--			idex_reset => ,
-- 			id_instruction => ,
--        	ex_instruction  => ,
--        	id_pc_plus_4 => ,
--       	ex_pc_plus_4 => ,
  	-- CONTROL signals
--        id_reg_dest => ,
--  	    id_branch => ,
--  	    id_mem_to_reg => ,
--  	    id_ALU_op => ,
--  	    id_mem_write => ,
--  	    id_ALU_src => ,
--  	    id_reg_write => ,
--  	    ex_reg_dest => ,
--  	    ex_branch => ,
--  	    ex_mem_to_reg => ,
--  	    ex_ALU_op => ,
--  	    ex_mem_write => ,
--  	    ex_ALU_src => ,
--  	    ex_reg_write => ,
  	-- END CONTROL signals
  	-- Register signals
--  		id_rs_data => ,
--  		id_rt_data => ,
--  		ex_rs_data => ,
--  		ex_rt_data => ,
--  		id_rs_sel => ,
--  		id_rt_sel => ,
--  		id_rd_sel => ,
--  		ex_rs_sel => ,
--  		ex_rt_sel => ,
--  		ex_rd_sel => ,
--  	-- END Register signals
--  		id_extended_immediate => ,
--  		ex_extended_immediate = );

--DESCRIPTION OF EACH SIGNAL FOR ID/EX
--  port(CLK           : in  std_logic;
--  		ex_flush, ex_stall, idex_reset : in std_logic;
-- 			id_instruction  : in std_logic_vector(31 downto 0); -- pass instruction along (useful for debugging)
--        	ex_instruction  : out std_logic_vector(31 downto 0);
--        	id_pc_plus_4 : in std_logic_vector(31 downto 0);
--       	ex_pc_plus_4 : out std_logic_vector(31 downto 0);
  	-- CONTROL signals
--        id_reg_dest   : in std_logic;
--  	    id_branch 	 : in std_logic;
--  	    id_mem_to_reg : in std_logic;
--  	    id_ALU_op 	 : in std_logic_vector(3 downto 0);
--  	    id_mem_write  : in std_logic;
--  	    id_ALU_src 	 : in std_logic;
--  	    id_reg_write  : in std_logic;
--  	    ex_reg_dest   : out std_logic;
--  	    ex_branch 	 : out std_logic;
--  	    ex_mem_to_reg : out std_logic;
--  	    ex_ALU_op 	 : out std_logic_vector(3 downto 0);
--  	    ex_mem_write  : out std_logic;
--  	    ex_ALU_src 	 : out std_logic;
--  	    ex_reg_write  : out std_logic;
--  	-- END CONTROL signals
--
--  	-- Register signals
--  		id_rs_data : in std_logic_vector(31 downto 0);
--  		id_rt_data : in std_logic_vector(31 downto 0);
--  		ex_rs_data : out std_logic_vector(31 downto 0);
--  		ex_rt_data : out std_logic_vector(31 downto 0);
--  		id_rs_sel : in std_logic_vector(4 downto 0);
--  		id_rt_sel : in std_logic_vector(4 downto 0);
--  		id_rd_sel : in std_logic_vector(4 downto 0);
--  		ex_rs_sel : out std_logic_vector(4 downto 0);
--  		ex_rt_sel : out std_logic_vector(4 downto 0);
--  		ex_rd_sel : out std_logic_vector(4 downto 0);
  	-- END Register signals
--  		id_extended_immediate : in std_logic_vector(31 downto 0);
--  		ex_extended_immediate : out std_logic_vector(31 downto 0)

--MEM/WB unit
--entity mem_wb is
--  port(CLK           : in  std_logic;
--		wb_flush, wb_stall, memwb_reset : in std_logic;
--		mem_instruction  : in std_logic_vector(31 downto 0); -- pass instruction along (useful for debugging)
--        wb_instruction  : out std_logic_vector(31 downto 0);
--        mem_pc_plus_4 : in std_logic_vector(31 downto 0);
--       	wb_pc_plus_4 : out std_logic_vector(31 downto 0);
  	-- CONTROL signals
--        mem_reg_dest   : in std_logic;
--  	    mem_mem_to_reg : in std_logic;
--  	    mem_reg_write  : in std_logic;
--  	    wb_reg_dest   : out std_logic;
--  	    wb_mem_to_reg : out std_logic;
--  	    wb_reg_write  : out std_logic;
  	-- END CONTROL signals

  	-- ALU signals
--		mem_ALU_out : in std_logic_vector(31 downto 0);
--		wb_ALU_out : out std_logic_vector(31 downto 0);
  	-- END ALU signals

  	-- Memory signals
--		mem_dmem_out : in std_logic_vector(31 downto 0);
--		wb_dmem_out : out std_logic_vector(31 downto 0);
  	-- END Memory signals

	-- Register signals
--  		mem_write_reg_sel : in std_logic_vector(4 downto 0);
--  		wb_write_reg_sel : out std_logic_vector(4 downto 0)
  	-- END Register signals
--  	    );

--entity ex_mem is
--  port(CLK           : in  std_logic;
--		mem_flush, mem_stall, exmem_reset : in std_logic;
--		ex_instruction  : in std_logic_vector(31 downto 0); -- pass instruction along (useful for debugging)
--        mem_instruction  : out std_logic_vector(31 downto 0);
--        ex_pc_plus_4 : in std_logic_vector(31 downto 0);
--       	mem_pc_plus_4 : out std_logic_vector(31 downto 0);

  	-- CONTROL signals
--        ex_reg_dest   : in std_logic;
--  	    ex_mem_to_reg : in std_logic;
--  	    ex_mem_write  : in std_logic;
--  	    ex_reg_write  : in std_logic;
--  	    mem_reg_dest   : out std_logic;
--  	    mem_mem_to_reg : out std_logic;
--  	    mem_mem_write  : out std_logic;
--  	    mem_reg_write  : out std_logic;
  	-- END CONTROL signals

  	-- ALU signals
--		ex_ALU_out : in std_logic_vector(31 downto 0);
--		mem_ALU_out : out std_logic_vector(31 downto 0);
  	-- END ALU signals

	-- Register signals
--		ex_rt_data : in std_logic_vector(31 downto 0);
--		mem_rt_data : out std_logic_vector(31 downto 0);
--  		ex_write_reg_sel : in std_logic_vector(4 downto 0); -- see the Reg. Dest. mux in the pipeline archteicture diagram
--  		mem_write_reg_sel : out std_logic_vector(4 downto 0)
  	-- END Register signals
--  	    );