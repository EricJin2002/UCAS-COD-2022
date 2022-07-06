#include "printf.h"
#include "trap.h"
#include "mul.h"
#include "div.h"
#include "perf_cnt.h"

#define FRAC_BIT 10

#define RD_ADDR 135106448
#define RD_SIZE_D0 1
#define RD_SIZE_D1 1
#define RD_SIZE_D2 28
#define RD_SIZE_D3 28

#define WEIGHT_ADDR 134217728
#define WEIGHT_SIZE_D0 20
#define WEIGHT_SIZE_D1 1
#define WEIGHT_SIZE_D2 5
#define WEIGHT_SIZE_D3 5

#define WR_ADDR 135108240
#define WR_SIZE_D0 1
#define WR_SIZE_D1 20
#define WR_SIZE_D2 12
#define WR_SIZE_D3 12

#define KERN_ATTR_CONV_PAD 0
#define KERN_ATTR_CONV_STRIDE 1
#define KERN_ATTR_POOL_PAD 0
#define KERN_ATTR_POOL_KERN_SIZE 2
#define KERN_ATTR_POOL_STRIDE 2

//MMIO register address of DNN accelerator
#define GPIO_START_ADDR    0x60030000
#define GPIO_DONE_ADDR     0x60030008

struct size_vec4
{
	unsigned d0;
	unsigned d1;
	unsigned d2;
	unsigned d3;
};

struct mem_addr
{
	unsigned rd_addr;
	unsigned weight_addr;
	unsigned wr_addr;
};

int mul(short a, short b)
{
#ifndef USE_MUL
	int ans = mul_ll(a, b);
#else
	int ans = a * b;
#endif
	return ans;
}

struct mem_addr addr = {RD_ADDR, WEIGHT_ADDR, WR_ADDR};
struct size_vec4 rd_size = {RD_SIZE_D0, RD_SIZE_D1, RD_SIZE_D2, RD_SIZE_D3};
struct size_vec4 wr_size = {WR_SIZE_D0, WR_SIZE_D1, WR_SIZE_D2, WR_SIZE_D3};
struct size_vec4 weight_size = {WEIGHT_SIZE_D0, WEIGHT_SIZE_D1, WEIGHT_SIZE_D2, WEIGHT_SIZE_D3};

struct size_vec4 conv_size;

extern char _binary_data_result_bin_start[];
extern char _binary_data_result_bin_size[];

void convolution()
{
	short *in = (short *)addr.rd_addr;
	short *weight = (short *)addr.weight_addr;
	short *out = (short *)addr.wr_addr;

	unsigned output_offset = 0;
	unsigned input_offset = 0;

	unsigned input_fm_w = rd_size.d3;
	unsigned input_fm_h = rd_size.d2;

	unsigned pad = KERN_ATTR_CONV_PAD;
	unsigned pad_len = pad << 1;

	unsigned conv_out_w = rd_size.d3 - weight_size.d3 + pad_len;
	unsigned conv_out_h = rd_size.d2 - weight_size.d2 + pad_len;

	unsigned stride = KERN_ATTR_CONV_STRIDE;

	conv_out_w = div(conv_out_w, stride);
	conv_out_h = div(conv_out_h, stride);

	conv_out_w++;
	conv_out_h++;

	conv_size.d0 = wr_size.d0;
	conv_size.d1 = wr_size.d1;
	conv_size.d2 = conv_out_h;
	conv_size.d3 = conv_out_w;

	//TODO: Please add your implementation here
	typedef short (*IN)[WEIGHT_SIZE_D1][input_fm_h][input_fm_w];
	typedef short (*WEIGHT)[WEIGHT_SIZE_D0][WEIGHT_SIZE_D1][mul(WEIGHT_SIZE_D2, WEIGHT_SIZE_D3) + 1];
	typedef short (*OUT)[WEIGHT_SIZE_D0][conv_out_h][conv_out_w];
	IN in_array = (IN)(in + input_offset);
	WEIGHT weight_array = (WEIGHT)weight;
	OUT out_array = (OUT)(out + output_offset);
	for(int no=0;no<WEIGHT_SIZE_D0;no++){
		for(int ni=0;ni<WEIGHT_SIZE_D1;ni++){
			for(int y=0;y<conv_out_h;y++){
				for(int x=0;x<conv_out_w;x++){
					if(ni==0){
						(*out_array)[no][y][x] = (*weight_array)[no][0][0];
					}
					int ih = mul(y, stride) - pad;
					int unshifted = 0;
					for(int ky=0;ky<WEIGHT_SIZE_D2;ky++,ih++){
						int iw = mul(x, stride) - pad;
						for(int kx=0;kx<WEIGHT_SIZE_D3;kx++,iw++){
							if(iw>=0 && iw<input_fm_w && ih>=0 && ih<input_fm_h){
								unshifted += mul(
								 	(*in_array)[ni][ih][iw],
								 	(*weight_array)[no][ni][mul(ky, WEIGHT_SIZE_D3) + kx + 1]
								);
							}
						}
					}
					(*out_array)[no][y][x] += unshifted >> FRAC_BIT;
				}
			}
		}
	}
}

void pooling()
{
	short *out = (short *)addr.wr_addr;

	unsigned output_offset = 0;
	unsigned input_offset = 0;

	unsigned input_fm_w = conv_size.d3;
	unsigned input_fm_h = conv_size.d2;

	unsigned pad = KERN_ATTR_POOL_PAD;
	unsigned pad_len = pad << 1;

	unsigned pad_w_test = conv_size.d3 - KERN_ATTR_POOL_KERN_SIZE;
	unsigned pad_h_test = conv_size.d2 - KERN_ATTR_POOL_KERN_SIZE;

	unsigned pool_out_w = pad_w_test + pad_len;
	unsigned pool_out_h = pad_h_test + pad_len;

	unsigned stride = KERN_ATTR_POOL_STRIDE;

	unsigned pad_w_test_remain = pad_w_test - mul(div(pad_w_test, stride), stride);
	unsigned pad_h_test_remain = pad_h_test - mul(div(pad_h_test, stride), stride);

	pool_out_w = div(pool_out_w, stride);
	pool_out_h = div(pool_out_h, stride);
	pool_out_w++;
	pool_out_h++;

	if ((!pad) && (pad_w_test_remain || pad_h_test_remain))
	{
		pool_out_w++;
		pool_out_h++;
	}

	//TODO: Please add your implementation here
	typedef short (*BEF_POOL)[WEIGHT_SIZE_D0][input_fm_h][input_fm_w];
	typedef short (*AFT_POOL)[WEIGHT_SIZE_D0][pool_out_h][pool_out_w];
	unsigned cache_offset = mul(WEIGHT_SIZE_D0, mul(input_fm_h, input_fm_w));
	BEF_POOL bef_array = (BEF_POOL)(out + input_offset);
	AFT_POOL aft_array = (AFT_POOL)(out + input_offset + cache_offset);
	for(int no=0;no<WEIGHT_SIZE_D0;no++){
		for(int y=0;y<pool_out_h;y++){
			for(int x=0;x<pool_out_w;x++){
				(*aft_array)[no][y][x] = 0x8000;
				int ih = mul(y, stride) - pad;
				for(int ky=0;ky<KERN_ATTR_POOL_KERN_SIZE;ky++,ih++){
					int iw = mul(x, stride) - pad; 
					for(int kx=0;kx<KERN_ATTR_POOL_KERN_SIZE;kx++,iw++){
						if(iw>=0 && iw<input_fm_w && ih>=0 && ih<input_fm_h){
							if((*aft_array)[no][y][x] < (*bef_array)[no][ih][iw]){
								(*aft_array)[no][y][x] = (*bef_array)[no][ih][iw];
							}
						}
					}
				}
			}
		}
	}
	int i = input_offset + cache_offset;
	int o = output_offset;
	for(int k=0;k<mul(WEIGHT_SIZE_D0, mul(pool_out_h, pool_out_w));k++,i++,o++){
		out[o]=out[i];
	}
}

#ifdef USE_HW_ACCEL
void launch_hw_accel()
{
	volatile int* gpio_start = (void*)(GPIO_START_ADDR);
	volatile int* gpio_done = (void*)(GPIO_DONE_ADDR);

	//TODO: Please add your implementation here
	(*gpio_start) |= 1;
	while(!((*gpio_done) & 1));
	(*gpio_start) &= 0;
}
#endif

int comparing()
{
	char *out = (char *)addr.wr_addr;
	char *result = (char *)_binary_data_result_bin_start;

#ifdef USE_HW_ACCEL
	int count = (int)_binary_data_result_bin_size + 
		    (16 - WR_SIZE_D3) * 2 * WR_SIZE_D2 * WR_SIZE_D1;
#else
	int count = (int)_binary_data_result_bin_size;
#endif

	for (int i = 0, j = 0; i < count; i++)
	{
#ifdef USE_HW_ACCEL
		int alignment = i & 0x0000001f;
		if (alignment >= (WR_SIZE_D3 << 1))
			continue;
#endif
		if (*(out + i) != *(result + j))
		{
			printf("Failed! at address %x and %x with data %x and %x\n", out + i, result + j, *(out + i), *(result + j));
			return 1;
		}
		j++;
	}

	printf("Passed!\n");
	return 0;
}

int main()
{
	Result res;
	bench_prepare(&res);

#ifdef USE_HW_ACCEL
	printf("Launching task...\n");
	launch_hw_accel();
#else
	printf("starting convolution\n");
	convolution();
	printf("starting pooling\n");
	pooling();
#endif

	int result = comparing();

	bench_done(&res);
	printf("======Hardware Performance Counter======\n");
    printf("Cycle Count:                %u\n", res.cnt[0]);
    printf("Instruction Count:          %u\n", res.cnt[1]);
    printf("Memory Read:                %u\n", res.cnt[2]);
    printf("Memory Write:               %u\n", res.cnt[3]);
    printf("Instruction Request Delay:  %u\n", res.cnt[4]);
    printf("Instruction Response Delay: %u\n", res.cnt[5]);
    printf("MemRead Request Delay:      %u\n", res.cnt[6]);
    printf("Read Data Delay:            %u\n", res.cnt[7]);
    printf("MemWrite Request Delay:     %u\n", res.cnt[8]);
    printf("Branch Count:               %u\n", res.cnt[9]);
    printf("Jump Count:                 %u\n", res.cnt[10]);
    printf("========================================\n");
	printf("benchmark finished\n");

	if (result == 0) {
		hit_good_trap();
	} else {
		nemu_assert(0);
	}
	
	return 0;
}
