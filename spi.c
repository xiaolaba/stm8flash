/* spi-based SWIM implementation
   (c) Andrzej Szombierski 2016 */

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <linux/spi/spidev.h>
#include <assert.h>
#include "spi.h"
#include "pgm.h"
#include "utils.h"

//#define DEBUG_SPI

#define SPI_DEVICE "/dev/spidev0.0"
#define RST_GPIO "25"
#define RISETIME_WORKAROUND 2

// equal to HSI clock
#define SPI_FREQMHZ 8

#define SPI_BITS1US 	SPI_FREQMHZ
#define SPI_BYTES1MS 	(1000*SPI_BITS1US/8)

#define SWIM_SRST 0
#define SWIM_ROTF 1
#define SWIM_WOTF 2

struct bitbuf {
	unsigned char *cur, *end;
	int bit;
};

static void bb_extend(struct bitbuf *bb)
{
	bb->cur++;
	bb->bit=0;
	assert(bb->cur < bb->end);
}

static inline void bb_put(struct bitbuf *bb, int bit)
{
	*bb->cur = (*bb->cur<<1) | (!!bit);
	bb->bit++;
	if(bb->bit == 8) 
		bb_extend(bb);
}

static inline int bb_get(struct bitbuf *bb)
{
	if(bb->cur == bb->end)
		return -1;

	int rv = !!(*bb->cur & 0x80);
	*bb->cur <<= 1;
	bb->bit++;
	if(bb->bit == 8) {
		bb->cur++;
		bb->bit=0;
	}
	return rv;
}

static void bb_flush(struct bitbuf *bb, int bit) {
	while(bb->bit)
		bb_put(bb, bit);
}

static void bb_init(struct bitbuf * bb, unsigned char *buf, int len)
{
	bb->cur = buf;
	bb->end = buf+len;
	bb->bit=0;
}

// low-speed SWIM bit
// ajusted for slow-ish rise time
static inline void ls_bit(struct bitbuf *bb, int bit)
{
	int i;
	for(i=0;i<(bit ? 2 : (20-RISETIME_WORKAROUND));i++)
		bb_put(bb, 1);
	for(i=0;i<(bit ? 20 : (2+RISETIME_WORKAROUND));i++)
		bb_put(bb, 0);
}

// space for low-speed SWIM bit
static inline void ls_space(struct bitbuf *bb, int n)
{
	int i;
	for(i=0;i<22*n;i++)
		bb_put(bb, 0);
}

static int decode_ls_bits(struct bitbuf *bb, int nbits, int *out)
{
	int i,j;
	int old=1,n=0;
	int count=0;
	*out = 0;
	while(nbits>0) {
		int b=bb_get(bb);
		if(b < 0)
			return count;

		if(b!=old) {
			old=b;
			if(b) {
				*out=(*out<<1) | ((n<9)?1:0);
				count++;
				nbits--;
			}
			n=0;
		}
		n++;
	}
	return count;
}



static void swim_cmd(struct bitbuf *bb, int cmd)
{
	int p = ((cmd>>2)^(cmd>>1)^cmd)&1;
	ls_bit(bb, 0); // host
	ls_bit(bb, cmd&4); // b2
	ls_bit(bb, cmd&2); // b1 
	ls_bit(bb, cmd&1); // b0
	ls_bit(bb, p); // bp
	ls_space(bb, 1); // ack
}

static void swim_byte(struct bitbuf *bb, int byte)
{
	int p = ((byte>>7)^(byte>>6)^(byte>>5)^(byte>>4)^(byte>>3)^(byte>>2)^(byte>>1)^byte)&1;
	ls_bit(bb, 0); // host
	ls_bit(bb, byte&128); // b7
	ls_bit(bb, byte&64); // b6
	ls_bit(bb, byte&32); // b5
	ls_bit(bb, byte&16); // b4
	ls_bit(bb, byte&8); // b3
	ls_bit(bb, byte&4); // b2
	ls_bit(bb, byte&2); // b1
	ls_bit(bb, byte&1); // b0
	ls_bit(bb, p); // bp
	ls_space(bb, 1); // ack
}

static void swim_rotf(struct bitbuf *bb, int address, int len)
{
	swim_cmd(bb, SWIM_ROTF);
	swim_byte(bb, len);
	swim_byte(bb, address>>16);
	swim_byte(bb, address>>8);
	swim_byte(bb, address);
	// send ack later
}

static void swim_wotf(struct bitbuf *bb, int address, int len, const unsigned char *data)
{
	swim_cmd(bb, SWIM_WOTF);
	swim_byte(bb, len);
	swim_byte(bb, address>>16);
	swim_byte(bb, address>>8);
	swim_byte(bb, address);
	for(;len>0;--len)
		swim_byte(bb, *data++);
}

static bool swim_ack_check(struct bitbuf *bb, int n_bytes)
{
	int rv, dec, i;
	dec = decode_ls_bits(bb, 6, &rv);
	if(dec != 6 || (rv&1) != 1) { // find ACK
		return false;
	}

	for(i=0;i<n_bytes;i++) {
		dec = decode_ls_bits(bb, 11, &rv);
		if(dec != 11 || (rv&1) != 1){  // find ACK
			return false;
		}
	}
	
	return true;
}

#ifdef DEBUG_SPI
void decode(unsigned char *ptr, int len, int x)
{
	int i,j;
	int old=1,n=0;
	for(i=0;i<len;i++) {
		int q=ptr[i]^x;
		for(j=0;j<8;j++) {
			int b=!!(q&0x80);
			if(b!=old) {
				old=b;
				if(b)
					putchar(n<9?'1':'0');
				else
					if(n>24)
						putchar('-');
				n=0;
			}
			n++;
			q<<=1;
		}
		if((i&7)==0)
			putchar('.');
	}
	printf("\n");
}

void dump(unsigned char *ptr, int len, int x)
{
	int i,j;
	for(i=0;i<len;i++) {
		int q=ptr[i]^x;
		for(j=0;j<8;j++) {
			putchar(q&0x80?'1':'0');
			q<<=1;
		}
	}
	printf("\n");
}
#endif

void spi_transact(int spi, unsigned char *buf, struct bitbuf * bb, unsigned char *rcvbuf, int rcvlen)
{
	memset(rcvbuf, 0, rcvlen);
	bb_flush(bb, 0);

	if(!rcvbuf) {
		rcvbuf = buf;
		rcvlen = bb->cur-buf;
	}
	assert(rcvlen >= bb->cur - buf);
	assert(bb->end - buf >= rcvlen);
	memset(bb->cur, 0, rcvlen-(bb->cur-buf));
	struct spi_ioc_transfer tf = {
		(__u64)(intptr_t)buf, (__u64)(intptr_t)rcvbuf,
		rcvlen,
	};
	ioctl(spi, SPI_IOC_MESSAGE(1), &tf);
#ifdef DEBUG_SPI
	printf("\n");
	//dump(buf, rcvlen, 0xff);
	//dump(rcvbuf, rcvlen, 0);
	printf("CMD >");
	decode(buf, bb->cur-buf, 0xff);
	printf("RSP >");
	decode(rcvbuf, rcvlen, 0);
	printf("\n");
#endif
}

void send_sync_sequence(int spi);
int spi_swim_read_byte(programmer_t *pgm, unsigned int start) 
{
	unsigned char sndbuf[256], rcvbuf[256];
	struct bitbuf bb;
	int retry = 3;
	while(retry > 0) {
		//printf("Read byte %x\n", start);
		bb_init(&bb, sndbuf, sizeof(sndbuf));
		swim_rotf(&bb, start, 1);
		spi_transact(((spi_context_t*)pgm->ctx)->spi_fd, sndbuf, &bb, rcvbuf, sizeof(rcvbuf));

		bb_init(&bb, rcvbuf, sizeof(rcvbuf));
		if(swim_ack_check(&bb, 4)) {
			int rv, dec;
			dec = decode_ls_bits(&bb, 100, &rv);
			int byte=0;
				
			if(dec == 10 && rv & 512) {
				byte = (rv>>1)&255;
				int p = ((byte>>7)^(byte>>6)^(byte>>5)^(byte>>4)^(byte>>3)^(byte>>2)^(byte>>1)^byte) & 1;
				if((rv&1)==p) {
					// ok
					bb_init(&bb, sndbuf, sizeof(sndbuf));
					ls_bit(&bb, 1); // send ack
					spi_transact(((spi_context_t*)pgm->ctx)->spi_fd, sndbuf, &bb, rcvbuf, sizeof(rcvbuf));
					return byte;

				} else {
					//fprintf(stderr, "byte = %x PARITY ERROR %x\n", byte, rv);
				} 
			} else {
				//fprintf(stderr, "read byte: decode error %d %x\n", dec, rv);
			}
		}

		fprintf(stderr, "(retrying read command)");
		--retry;
		send_sync_sequence(((spi_context_t*)pgm->ctx)->spi_fd); // the whole sync sequence is not required, but this makes things simpler
	}
	
	fprintf(stderr, "Fatal SWIM Communications error\n");
	return -1;
}

bool spi_swim_write_byte(programmer_t *pgm, unsigned char byte, unsigned int start) 
{
	unsigned char sndbuf[256], rcvbuf[256];
	struct bitbuf bb;
	int retry=3;
	while(retry > 0) {
		bb_init(&bb, sndbuf, sizeof(sndbuf));
		swim_wotf(&bb, start, 1, &byte);
		spi_transact(((spi_context_t*)pgm->ctx)->spi_fd, sndbuf, &bb, rcvbuf, sizeof(rcvbuf));
		
		bb_init(&bb, rcvbuf, sizeof(rcvbuf));
		if(swim_ack_check(&bb, 4+1)) 
			return true;

		fprintf(stderr, "(retrying write command)");
		--retry;
		send_sync_sequence(((spi_context_t*)pgm->ctx)->spi_fd); // the whole sync sequence is not required, but this makes things simpler
	}

	fprintf(stderr, "Fatal SWIM Communications error\n");
	return false;
}

bool spi_init_session(programmer_t *pgm) {
	return spi_swim_write_byte(pgm, 0xa0, 0x7f80); // mov 0x0a, SWIM_CSR2 ;; Init SWIM
	//spi_swim_write_byte(pgm, 0xb0, 0x7f80);
	//spi_swim_write_byte(pgm, 0xb4, 0x7f80);
}

bool spi_finish_session(programmer_t *pgm) {
	return spi_swim_write_byte(pgm, 0xb6, 0x7f80);
}

void send_sync_sequence(int spi) {
	unsigned char sndbuf[32*SPI_BITS1US/8 + SPI_BYTES1MS * 7];
	unsigned char rcvbuf[sizeof(sndbuf)];
	int q=0;
	int i,j;
	for(i=0;i<16*SPI_BITS1US/8;i++)
		sndbuf[q++] = 0; // low init

	for(i=0;i<16*SPI_BITS1US/8;i++) // 16us
		sndbuf[q++] = 0xff; // high 16us

	for(i=0;i<4;i++) {
		for(j=0;j<SPI_BYTES1MS/2;j++)
			sndbuf[q++]=0; // high .5ms
		for(;j<SPI_BYTES1MS;j++)
			sndbuf[q++]=0xff; // low .5ms
	}
	for(i=0;i<2;i++) {
		for(j=0;j<SPI_BYTES1MS/4;j++)
			sndbuf[q++]=0; // high .25ms
		for(;j<2*SPI_BYTES1MS/4;j++)
			sndbuf[q++]=0xff; // low .25ms
		for(;j<3*SPI_BYTES1MS/4;j++)
			sndbuf[q++]=0; // high .25ms
		for(;j<SPI_BYTES1MS;j++)
			sndbuf[q++]=0xff; // low .25ms
	}
	for(i=0;i<SPI_BYTES1MS/2;i++)
		sndbuf[q++]=0;
	
	assert(q < sizeof(sndbuf));
	
	struct spi_ioc_transfer tf = {
		(__u64)(intptr_t)sndbuf, (__u64)(intptr_t)rcvbuf,
		q,
	};
	ioctl(spi, SPI_IOC_MESSAGE(1), &tf);
	//dump(sndbuf, q, 0xff);
	//dump(rcvbuf, q, 0);

	// TODO: find sync pulse
}

bool spi_open(programmer_t *pgm) {
	int spi = open("/dev/spidev0.0", O_RDWR);
	if(spi < 0)
		return false;

	int q=1000000*SPI_FREQMHZ;
	ioctl(spi, SPI_IOC_WR_MAX_SPEED_HZ, &q);
	int fd = open("/sys/class/gpio/gpio" RST_GPIO "/direction", O_RDWR);
	if(fd < 0) {
		fd = open("/sys/class/gpio/export", O_WRONLY);
		if(fd >= 0) {
			write(fd, RST_GPIO, strlen(RST_GPIO));
			close(fd);
		}

		fd = open("/sys/class/gpio/gpio" RST_GPIO "/direction", O_RDWR);
	} 

	if(fd < 0) {
		fprintf(stderr, "Can't open gpio" RST_GPIO "/direction: %m\n");
		close(spi);
		return false;
	}
	
	// RESET
	if(write(fd, "low", 3) != 3) {
		fprintf(stderr, "Can't write to gpio" RST_GPIO "/direction: %m\n");
		close(spi);
		close(fd);
		return false;
	}

	// sync sequence
	send_sync_sequence(spi);

	spi_context_t *ctx = malloc(sizeof(spi_context_t));
	ctx->spi_fd = spi;
	pgm->ctx = ctx;
	
	spi_swim_write_byte(pgm, 0xa0, 0x7f80); // mov 0x0a, SWIM_CSR2 ;; Init SWIM

	// de-reset
	fd = open("/sys/class/gpio/gpio" RST_GPIO "/direction", O_RDWR);
	if(fd >= 0) {
		write(fd, "in", 2);
		close(fd);
	}
	usleep(1000);

	return(true);
}

void spi_close(programmer_t *pgm) {
	// de-reset
	int fd = open("/sys/class/gpio/gpio" RST_GPIO "/direction", O_RDWR);
	if(fd >= 0) {
		write(fd, "in", 2);
		close(fd);
	}

	spi_context_t *ctx = pgm->ctx;
	close(ctx->spi_fd);
	free(pgm->ctx);
}

void spi_srst(programmer_t *pgm) {
	// RESET
	int fd = open("/sys/class/gpio/gpio" RST_GPIO "/direction", O_RDWR);
	if(fd >= 0) {
		write(fd, "low", 3);
		close(fd);
	}

	usleep(10000);

	// de-reset
	fd = open("/sys/class/gpio/gpio" RST_GPIO "/direction", O_RDWR);
	if(fd >= 0) {
		write(fd, "in", 2);
		close(fd);
	}
}

void spi_swim_srst(programmer_t *pgm) {
	unsigned char sndbuf[256], rcvbuf[256];
	struct bitbuf bb;
	bb_init(&bb, sndbuf, sizeof(sndbuf));
	swim_cmd(&bb, SWIM_SRST);
	spi_transact(((spi_context_t*)pgm->ctx)->spi_fd, sndbuf, &bb, rcvbuf, sizeof(rcvbuf));
}

int spi_swim_read_range(programmer_t *pgm, const stm8_device_t *device, unsigned char *buffer, unsigned int start, unsigned int length) {
	unsigned char buf[4];
	DEBUG_PRINT("spi_swim_read_range\n");
	spi_init_session(pgm);
	spi_swim_write_byte(pgm, 0x00, device->regs.CLK_CKDIVR); // mov 0x00, CLK_DIVR
	memset(buffer, 0xff, length);

	int nr=0, retry=3;
	while(length > 0) {
		unsigned char sndbuf[4096], rcvbuf[4096];
		struct bitbuf bb;
		bb_init(&bb, sndbuf, sizeof(sndbuf));
		int l = length;
		if(l > 64) l = 64;
		swim_rotf(&bb, start, l);
		ls_space(&bb, 1); // remote ack

		int i;
		for(i=0;i<l;i++) {
			ls_space(&bb, 10 /* response */ + 2 /*margin */);
			ls_bit(&bb, 1); // my preemptive ack
		}

		spi_transact(((spi_context_t*)pgm->ctx)->spi_fd, sndbuf, &bb, rcvbuf, sizeof(rcvbuf));
		bb_init(&bb, rcvbuf, sizeof(rcvbuf));

		bool ok = true;
		if(swim_ack_check(&bb, 4)) {
			for(i=0;i<l;i++){
				int rv, dec;
				dec = decode_ls_bits(&bb, 10, &rv);
				if(dec == 10 && rv & 512) {
					int byte = (rv>>1)&255;
					int p = ((byte>>7)^(byte>>6)^(byte>>5)^(byte>>4)^(byte>>3)^(byte>>2)^(byte>>1)^byte) & 1;
					if((rv&1)==p) {
						*buffer++ = byte;
						nr++;
					} else {
						// fprintf(stderr, "byte %d = %x PARITY ERROR %x\n", i, byte, rv);
						ok = false;
						break;
					} 
				} else {
					//fprintf(stderr, "read range: decode error %d %x\n", dec, rv);
					ok = false;
					break;
				}
				
				// my ack
				dec = decode_ls_bits(&bb, 1, &rv);
				if(dec != 1 || rv != 1) {
					ok = false;
					break;
				}
			}
		} else {
			ok = false;
		}

		if(ok) {
			start += l;
			length -= l;
			retry = 3;

		} else {
			if(retry == 0) {
				fprintf(stderr, "Fatal SWIM Communications error\n");
				break;
			}

			fprintf(stderr, "(retrying read command)");
			--retry;
			send_sync_sequence(((spi_context_t*)pgm->ctx)->spi_fd); // the whole sync sequence is not required, but this makes things simpler
		}
	}

	spi_finish_session(pgm);
	return(nr);
}

int spi_swim_write_block(programmer_t *pgm, unsigned char *buffer,
			    unsigned int start,
			    unsigned int length
			    ) 
{
	unsigned char sndbuf[4096];
	struct bitbuf bb;
	bb_init(&bb, sndbuf, sizeof(sndbuf));
	swim_wotf(&bb, start, length, buffer);
	ls_space(&bb, 2); // for remote ack
	spi_transact(((spi_context_t*)pgm->ctx)->spi_fd, sndbuf, &bb, 0, 0);

	return(0);
}

int spi_swim_write_range(programmer_t *pgm, const stm8_device_t *device, unsigned char *buffer, unsigned int start, unsigned int length, const memtype_t memtype) {
	int i;
	spi_init_session(pgm);
//	spi_swim_write_byte(pgm, 0x00, device->regs.CLK_CKDIVR);
	if(memtype == FLASH || memtype == EEPROM || memtype == OPT) {
		spi_swim_write_byte(pgm, 0x00, device->regs.FLASH_IAPSR);
	}
	if(memtype == FLASH) {
		spi_swim_write_byte(pgm, 0x56, device->regs.FLASH_PUKR);
		spi_swim_write_byte(pgm, 0xae, device->regs.FLASH_PUKR);
	}
	if(memtype == EEPROM || memtype == OPT) {
		spi_swim_write_byte(pgm, 0xae, device->regs.FLASH_DUKR);
		spi_swim_write_byte(pgm, 0x56, device->regs.FLASH_DUKR);
	}
	if(memtype == FLASH || memtype == EEPROM || memtype == OPT) {
		spi_swim_write_byte(pgm, 0x56, device->regs.FLASH_IAPSR);
	}
	int flash_block_size = device->flash_block_size;
	for(i = 0; i < length; i+=flash_block_size) {
		unsigned char block[128];
		memset(block, 0, sizeof(block));
		int block_size = length - i;
		if(block_size > flash_block_size)
			block_size = flash_block_size;
		DEBUG_PRINT("Writing block %04x with size %d\n", start+i, block_size);
		memcpy(block, buffer+i, block_size);
		if(block_size < flash_block_size) {
			DEBUG_PRINT("Padding block %04x with %d zeroes\n",
				    start+i,
				    flash_block_size - block_size);
			block_size = flash_block_size;
		}
		if(memtype == FLASH || memtype == EEPROM || memtype == OPT) {
			spi_swim_write_byte(pgm, 0x01, device->regs.FLASH_CR2);
			if(device->regs.FLASH_NCR2 != 0) { // Device have FLASH_NCR2 register
				spi_swim_write_byte(pgm, 0xFE, device->regs.FLASH_NCR2);
			}
		}
		int result = spi_swim_write_block(pgm, block, start + i, block_size);
		if(result)
			fprintf(stderr, "Write error\n");

		bool fail=false;
		for(;;) {
			int iapsr = spi_swim_read_byte(pgm, device->regs.FLASH_IAPSR);
			if(iapsr < 0) {
				fprintf(stderr, "Transfer error\n");
				fail=true;
				break;
			}
			if(iapsr & 1) {
				fprintf(stderr, "Write error %x (protected?)\n", iapsr);
				fail=true;
				break;
			}

			if(iapsr & 4) {
				break;
			}
		}

		if(fail) break;

	}
	if(memtype == FLASH || memtype == EEPROM || memtype == OPT) {
		spi_swim_write_byte(pgm, 0x56, device->regs.FLASH_IAPSR);
	}
	spi_finish_session(pgm);
	return(i);
}
