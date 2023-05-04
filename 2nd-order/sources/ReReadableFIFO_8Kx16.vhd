--
--    Copyright (C) 2018-2023 by Lars Wienbrandt,
--    Institute of Clinical Molecular Biology, Kiel University
--    
--    This file is part of HybridGWAIS-FPGA.
--
--    HybridGWAIS-FPGA is free software: you can redistribute it and/or modify
--    it under the terms of the GNU General Public License as published by
--    the Free Software Foundation, either version 3 of the License, or
--    (at your option) any later version.
--
--    HybridGWAIS-FPGA is distributed in the hope that it will be useful,
--    but WITHOUT ANY WARRANTY; without even the implied warranty of
--    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--    GNU General Public License for more details.
--
--    You should have received a copy of the GNU General Public License
--    along with HybridGWAIS-FPGA. If not, see <https://www.gnu.org/licenses/>.
--
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- This FWFT FIFO can be filled with up 1k 16bit words. 
-- Reading a word from the FIFO simply inserts it again, such that all data in the FIFO 
-- will be read again and again. The FIFO has to be reset explicitly
-- to remove the data. The same applies to reset the full flag.
-- The empty flag is asserted only after a reset if there are no words in the FIFO. The
-- output is undefined in that case.
-- The FIFO has an enable/rnw interface which makes it impossible to read and write data
-- at the same time.
entity ReReadableFIFO_8Kx16 is
   port(
      clk       : in  std_logic;
      reset     : in  std_logic;

      d_in      : in  std_logic_vector(15 downto 0);
      d_out     : out std_logic_vector(15 downto 0);

      en_in     : in  std_logic;
      rnw_in    : in  std_logic;        -- 1 read, 0 write

      empty_out : out std_logic;
      full_out  : out std_logic
   );
end ReReadableFIFO_8Kx16;

architecture Behavioral of ReReadableFIFO_8Kx16 is

   signal fifo_din    : std_logic_vector(15 downto 0);
   signal fifo_dout   : std_logic_vector(15 downto 0);
   signal fifo_wr_en  : std_logic := '0';
   signal fifo_rd_en  : std_logic := '0';
   signal fifo_empty  : std_logic;
   signal fifo_amfull : std_logic;
   signal fifo_wr_rst_busy : std_logic;
   signal fifo_rd_rst_busy : std_logic;

begin
   fifo_i : entity work.FIFO_FWFT_8Kx16_Wrapper
      port map(
         clk         => clk,
         srst        => reset,
         din         => fifo_din,
         wr_en       => fifo_wr_en,
         rd_en       => fifo_rd_en,
         dout        => fifo_dout,
         full        => open,
         almost_full => fifo_amfull,
         empty       => fifo_empty,
         wr_rst_busy => fifo_wr_rst_busy,
         rd_rst_busy => fifo_rd_rst_busy
      );
   -- "full" is left open and "almost_full" is used instead to ensure a read datum can be written into the FIFO again

   -- put data back into FIFO when reading
   fifo_din   <= d_in when rnw_in = '0' else fifo_dout;
   -- write if written from outside on non-full FIFO, or if read from non-empty FIFO
   fifo_wr_en <= en_in and ((not rnw_in and not fifo_amfull) or (rnw_in and not fifo_empty));
   -- read if read from non-empty FIFO
   fifo_rd_en <= en_in and rnw_in and not fifo_empty;

   d_out     <= fifo_dout;
   empty_out <= fifo_empty;
   full_out  <= fifo_amfull;

end Behavioral;

