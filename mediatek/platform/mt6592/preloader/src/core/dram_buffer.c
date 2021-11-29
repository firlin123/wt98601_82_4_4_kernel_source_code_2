#include "dram_buffer.h"
#include "typedefs.h"
#include "emi.h"

#define MOD "[Dram_Buffer]"
#define DRAM_BASE 0x80000000

dram_buf_t* g_dram_buf = 0;

void init_dram_buffer(){
#if !CFG_FPGA_PLATFORM
	u32 dram_rank_size[4] = {0,0,0,0};
	u32 dram_size = 0;
	u32 structure_size = sizeof(dram_buf_t);
	/*get memory size*/
	get_dram_rank_size(dram_rank_size);
	dram_size= dram_rank_size[0] + dram_rank_size[1] + dram_rank_size[2] + dram_rank_size[3];
	print("%s dram size:%d \n" ,MOD, dram_size);
	print("%s structure size: %d \n" ,MOD, structure_size);
    print("%s structure size2: %d \n" ,MOD, sizeof(part_hdr_t));
	/*allocate dram_buf*/
	g_dram_buf = DRAM_BASE  + dram_size - (5*1024*1024);
#else
    g_dram_buf = 0x80C00000;

#endif
}

