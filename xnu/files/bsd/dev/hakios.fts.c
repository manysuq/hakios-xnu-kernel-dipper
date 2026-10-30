/*
 * The touch screen of the Xiaomi Mi 8 (dipper): an STMicroelectronics FingerTipS controller
 * (st_fts, "st,fts" in the phone's device tree, address 0x49) on the I2C serial engine 14.
 * It feeds /dev/hakios-touch (sys/hakios.input.h), like the Goodix driver does on redwood.
 *
 * Every address below is the phone's:
 *
 *   I2C       qupv3_se14_i2c at 0xa98000 (0x4000 bytes), serial engine 6 of the second QUPV3
 *             wrapper; data on GPIO 33 and clock on GPIO 34, both in the "qup14" function
 *   clocks    GCC at 0x100000: the voter 0x5200c (bits 20 and 21 for the wrapper's AHB clocks,
 *             28 for this SE), the SE's root clock at 0x18738 and its branch at 0x18734
 *   pins      TLMM at 0x340000: GPIO 31 interrupt (low while the chip has events), GPIO 32 reset
 *             (low in reset, high while running)
 *   chip      commands are one byte (0x80 read information, 0x85 read one event, 0x91 sleep out,
 *             0x93 multi-touch sense on, 0xa1 clear the event stack); an event is eight bytes:
 *             [0] = 0x03 enter / 0x04 leave / 0x05 motion in the low nibble and the finger in
 *             the high one, [1] = x low, [2] = x high bits in the low nibble and y low bits in
 *             the high one, [3] = y high; the panel reports 1080 x 2248
 *
 * The GENI serial engine is put into FIFO mode and polled; no interrupt is taken. A kernel
 * thread looks at the interrupt line every touch_poll_us microseconds (8000 by default), reads
 * the events while it is low, and posts each finger's position to hakios_touch_post(). The
 * controller's supplies (the display's 1.8 V rail and PM8998 L28) are left as the firmware and
 * Android's boot loader set them: both are marked "regulator-boot-on" in the device tree.
 *
 * Boot-arg touchscreen=2 selects this driver explicitly; the loader also passes "dipper=1" on this
 * phone, which selects it automatically (touchscreen=1 stays the Goodix/FocalTech driver of
 * redwood). touch_max_x=N, touch_max_y=N and touch_flip=1|2|3 adjust the panel if needed.
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.input.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern kern_return_t thread_terminate(thread_t thread);
extern void hakios_tl(const char *format, ...) __printflike(1, 2);

static vm_offset_t tlmm, gcc, se;

static inline uint32_t
rd(vm_offset_t address)
{
	return *(volatile uint32_t *)address;
}

static inline void
wr(vm_offset_t address, uint32_t value)
{
	*(volatile uint32_t *)address = value;
}

static void
sleep_us(uint32_t us)
{
	delay_for_interval(us, NSEC_PER_USEC);
}

static void
sleep_ms(uint32_t ms)
{
	delay_for_interval(ms, NSEC_PER_MSEC);
}

/* ---- pins (TLMM) ------------------------------------------------------------------------------- */

#define TLMM_PHYS               0x03400000UL
#define TLMM_SIZE               0x00100000UL
#define GPIO_CFG(n)             (tlmm + 0x1000UL * (n))         /* [1:0] pull, [5:2] function, [8:6] drive, [9] output */
#define GPIO_IO(n)              (tlmm + 0x1000UL * (n) + 4)     /* bit 0 in, bit 1 out */
#define PULL_NONE               0
#define PULL_UP                 3

#define PIN_INT                 31      /* low = the chip has events */
#define PIN_RESET               32      /* low = in reset */
#define PIN_SDA                 33
#define PIN_SCL                 34
#define PIN_FUNCTION_QUP14      2

static void
gpio_configure(unsigned pin, unsigned function, unsigned pull, unsigned ma, bool output)
{
	wr(GPIO_CFG(pin), pull | function << 2 | (ma / 2 - 1) << 6 | (output ? 1u << 9 : 0));
}

static void
gpio_set(unsigned pin, bool high)
{
	wr(GPIO_IO(pin), high ? 2 : 0);
}

static bool
gpio_get(unsigned pin)
{
	return rd(GPIO_IO(pin)) & 1;
}

static void
pins_init(void)
{
	/* the level first, so that the reset pin never drives the wrong one while it changes function */
	gpio_set(PIN_RESET, true);
	gpio_configure(PIN_INT, 0, PULL_UP, 2, false);
	gpio_configure(PIN_RESET, 0, PULL_UP, 2, true);
	gpio_configure(PIN_SDA, PIN_FUNCTION_QUP14, PULL_NONE, 2, false);
	gpio_configure(PIN_SCL, PIN_FUNCTION_QUP14, PULL_NONE, 2, false);
}

/* ---- clocks (GCC) ------------------------------------------------------------------------------- */

#define GCC_PHYS                0x00100000UL
#define GCC_SIZE                0x00060000UL
#define GCC_VOTE_WRAP1          (gcc + 0x5200c)         /* the application processor's votes */
#define VOTE_WRAP1_M_AHB        (1u << 20)
#define VOTE_WRAP1_S_AHB        (1u << 21)
#define VOTE_WRAP1_SE6          (1u << 28)
#define GCC_QUPV3_WRAP1_SE6_CMD (gcc + 0x18738)         /* bit 0: take the new configuration */
#define GCC_QUPV3_WRAP1_SE6_CFG (gcc + 0x1873c)         /* [4:0] divider, [10:8] source, [13:12] mode */
#define GCC_QUPV3_WRAP1_SE6_CBCR (gcc + 0x18734)
#define GCC_QUPV3_WRAP1_M_AHB_CBCR (gcc + 0x1800c)
#define GCC_QUPV3_WRAP1_S_AHB_CBCR (gcc + 0x18010)
#define CLOCK_OFF               (1u << 31)

static bool
clocks_on(void)
{
	/* the crystal (19.2 MHz), undivided: enough for a 400 kHz bus and the GENI FIFOs */
	wr(GCC_QUPV3_WRAP1_SE6_CFG, 0);
	wr(GCC_QUPV3_WRAP1_SE6_CMD, rd(GCC_QUPV3_WRAP1_SE6_CMD) | 1);
	for (int i = 0; i < 500 && (rd(GCC_QUPV3_WRAP1_SE6_CMD) & 1); i++) {
		sleep_us(10);
	}
	if (rd(GCC_QUPV3_WRAP1_SE6_CMD) & 1) {
		printf("fts: the serial engine's clock does not take its configuration (%#x)\n", rd(GCC_QUPV3_WRAP1_SE6_CMD));
		return false;
	}
	wr(GCC_VOTE_WRAP1, rd(GCC_VOTE_WRAP1) | VOTE_WRAP1_M_AHB | VOTE_WRAP1_S_AHB | VOTE_WRAP1_SE6);
	for (int i = 0; i < 1000; i++) {
		if (!((rd(GCC_QUPV3_WRAP1_M_AHB_CBCR) | rd(GCC_QUPV3_WRAP1_S_AHB_CBCR) |
		    rd(GCC_QUPV3_WRAP1_SE6_CBCR)) & CLOCK_OFF)) {
			return true;
		}
		sleep_us(10);
	}
	printf("fts: clocks stay off: %#x %#x %#x\n", rd(GCC_QUPV3_WRAP1_M_AHB_CBCR), rd(GCC_QUPV3_WRAP1_S_AHB_CBCR),
	    rd(GCC_QUPV3_WRAP1_SE6_CBCR));
	return false;
}

/* ---- I2C through the serial engine (GENI) ---------------------------------------------------------- */

#define SE_PHYS                 0x00a98000UL
#define SE_SIZE                 0x4000UL
#define GENI_FORCE_DEFAULT      (se + 0x020)
#define GENI_OUTPUT_CTRL        (se + 0x024)
#define GENI_CGC_CTRL           (se + 0x028)
#define GENI_STATUS             (se + 0x040)    /* bit 0: a command is running */
#define GENI_SER_M_CLK_CFG      (se + 0x048)    /* bit 0 enable, [15:4] divider */
#define GENI_IF_DISABLE_RO      (se + 0x064)    /* bit 0: no FIFO interface */
#define GENI_FW_REVISION_RO     (se + 0x068)    /* [15:8] protocol: 2 = I2C */
#define GENI_CLK_SEL            (se + 0x07c)
#define GENI_BYTE_GRAN          (se + 0x254)
#define GENI_DMA_MODE_EN        (se + 0x258)
#define GENI_TX_PACKING_CFG0    (se + 0x260)
#define GENI_TX_PACKING_CFG1    (se + 0x264)
#define SE_I2C_TX_TRANS_LEN     (se + 0x26c)
#define SE_I2C_RX_TRANS_LEN     (se + 0x270)
#define SE_I2C_SCL_COUNTERS     (se + 0x278)
#define GENI_RX_PACKING_CFG0    (se + 0x284)
#define GENI_RX_PACKING_CFG1    (se + 0x288)
#define GENI_M_CMD0             (se + 0x600)    /* [31:27] operation, then the I2C parameters */
#define GENI_M_CMD_CTRL         (se + 0x604)    /* bit 2 cancel, bit 1 abort */
#define GENI_M_IRQ_STATUS       (se + 0x610)
#define GENI_M_IRQ_EN           (se + 0x614)
#define GENI_M_IRQ_CLEAR        (se + 0x618)
#define GENI_S_IRQ_CLEAR        (se + 0x648)
#define GENI_TX_FIFO            (se + 0x700)
#define GENI_RX_FIFO            (se + 0x780)
#define GENI_TX_FIFO_STATUS     (se + 0x800)    /* [27:0] words */
#define GENI_RX_FIFO_STATUS     (se + 0x804)    /* bit 31 last, [30:28] bytes of the last word, [24:0] words */
#define GENI_TX_WATERMARK       (se + 0x80c)
#define GENI_RX_WATERMARK       (se + 0x810)
#define GENI_RX_RFR_WATERMARK   (se + 0x814)
#define SE_DMA_TX_IRQ_CLR       (se + 0xc44)
#define SE_DMA_RX_IRQ_CLR       (se + 0xd44)
#define SE_GSI_EVENT_EN         (se + 0xe18)
#define SE_IRQ_EN               (se + 0xe1c)
#define SE_HW_PARAM_0           (se + 0xe24)    /* [21:16] TX FIFO depth in words */
#define SE_DMA_GENERAL_CFG      (se + 0xe30)

#define I2C_OPCODE_WRITE        (1u << 27)
#define I2C_OPCODE_READ         (2u << 27)
#define I2C_STOP_STRETCH        (1u << 2)       /* hold SCL low after this message: another follows */
#define M_CMD_DONE              (1u << 0)
#define M_CMD_CANCEL            (1u << 4)
#define M_CMD_ABORT             (1u << 5)
#define M_ERRORS                ((1u << 1) | (1u << 2) | (1u << 3) | (1u << 9) | (1u << 10) | \
				 (1u << 12) | (1u << 13) | (1u << 24) | (1u << 25) | (1u << 28) | (1u << 29))

#define FTS_ADDRESS             0x49
#define FTS_PARAMS              (FTS_ADDRESS << 9)

static uint32_t fifo_depth;

static bool
geni_i2c_init(void)
{
	if (((rd(GENI_FW_REVISION_RO) >> 8) & 0xff) != 2) {
		printf("fts: the serial engine has no I2C firmware (%#x)\n", rd(GENI_FW_REVISION_RO));
		return false;
	}
	if (rd(GENI_IF_DISABLE_RO) & 1) {
		printf("fts: the serial engine's FIFO interface is disabled\n");
		return false;
	}
	fifo_depth = (rd(SE_HW_PARAM_0) >> 16) & 0x3f;
	if (fifo_depth < 2) {
		printf("fts: the serial engine reports no usable FIFO (%#x)\n", rd(SE_HW_PARAM_0));
		return false;
	}
	if (fifo_depth > 64) {
		fifo_depth = 64;
	}

	wr(SE_GSI_EVENT_EN, 0);
	wr(GENI_M_IRQ_CLEAR, 0xffffffff);
	wr(GENI_S_IRQ_CLEAR, 0xffffffff);
	wr(SE_DMA_TX_IRQ_CLR, 0xffffffff);
	wr(SE_DMA_RX_IRQ_CLR, 0xffffffff);
	wr(GENI_CGC_CTRL, rd(GENI_CGC_CTRL) | 0x7f);
	wr(SE_DMA_GENERAL_CFG, rd(SE_DMA_GENERAL_CFG) | 0xf);
	wr(GENI_OUTPUT_CTRL, 0x7f);
	wr(GENI_FORCE_DEFAULT, 1);
	wr(GENI_DMA_MODE_EN, rd(GENI_DMA_MODE_EN) & ~1u);
	wr(GENI_RX_WATERMARK, fifo_depth - 1);
	wr(GENI_RX_RFR_WATERMARK, fifo_depth);
	wr(GENI_TX_WATERMARK, 0);
	/* the status bits we poll follow these enables; the interrupt itself stays masked at the GIC */
	wr(SE_IRQ_EN, rd(SE_IRQ_EN) | 0xf);
	wr(GENI_M_IRQ_EN, rd(GENI_M_IRQ_EN) | 0x7fc0007f);

	/* eight bits, most significant first, four bytes in a FIFO word (as Linux does for GENI I2C) */
	wr(GENI_TX_PACKING_CFG0, 0x0007f8fe);
	wr(GENI_TX_PACKING_CFG1, 0x000ffefe);
	wr(GENI_RX_PACKING_CFG0, 0x0007f8fe);
	wr(GENI_RX_PACKING_CFG1, 0x000ffefe);
	wr(GENI_BYTE_GRAN, 0);

	/* 400 kHz from the 19.2 MHz crystal: divider 2, 5 high and 12 low cycles, 24 in all */
	wr(GENI_CLK_SEL, 0);
	wr(GENI_SER_M_CLK_CFG, 2 << 4 | 1);
	wr(SE_I2C_SCL_COUNTERS, 5u << 20 | 12u << 10 | 24);
	printf("fts: I2C serial engine ready: protocol %u, FIFO %u words\n", (rd(GENI_FW_REVISION_RO) >> 8) & 0xff, fifo_depth);
	return true;
}

static bool
geni_wait_idle(void)
{
	for (int i = 0; i < 2000; i++) {
		if (!(rd(GENI_STATUS) & 1)) {
			return true;
		}
		sleep_us(10);
	}
	return false;
}

static void
geni_recover(void)
{
	wr(GENI_M_CMD_CTRL, 1u << 2);
	for (int i = 0; i < 2000 && !(rd(GENI_M_IRQ_STATUS) & M_CMD_CANCEL); i++) {
		sleep_us(100);
	}
	if (!(rd(GENI_M_IRQ_STATUS) & M_CMD_CANCEL)) {
		wr(GENI_M_CMD_CTRL, 1u << 1);
		for (int i = 0; i < 2000 && !(rd(GENI_M_IRQ_STATUS) & M_CMD_ABORT); i++) {
			sleep_us(100);
		}
	}
	for (int i = 0; i < 64 && (rd(GENI_RX_FIFO_STATUS) & 0x01ffffff); i++) {
		(void)rd(GENI_RX_FIFO);
	}
	wr(GENI_M_IRQ_CLEAR, 0xffffffff);
}

/*
 * One write, with the bus held at the end when a read follows (STOP_STRETCH). The FIFO is deep
 * enough for every frame this driver sends (at most eight bytes), so it is filled at once.
 */
static bool
geni_i2c_write(const uint8_t *data, uint32_t length, bool hold)
{
	static unsigned complaints;
	uint32_t written = 0, status = 0;
	uint64_t deadline;

	if (length == 0 || length > fifo_depth * 4) {
		return false;
	}
	if (!geni_wait_idle()) {
		geni_recover();
		return false;
	}
	wr(GENI_M_IRQ_CLEAR, 0xffffffff);
	wr(SE_I2C_TX_TRANS_LEN, length);
	wr(GENI_M_CMD0, I2C_OPCODE_WRITE | FTS_PARAMS | (hold ? I2C_STOP_STRETCH : 0));
	while (written < length) {
		uint32_t word = 0;

		for (int i = 0; i < 4 && written < length; i++, written++) {
			word |= (uint32_t)data[written] << (8 * i);
		}
		wr(GENI_TX_FIFO, word);
	}
	clock_interval_to_deadline(50, NSEC_PER_MSEC, &deadline);
	while (mach_absolute_time() < deadline) {
		status = rd(GENI_M_IRQ_STATUS);
		if (status & M_ERRORS) {
			break;
		}
		if (status & M_CMD_DONE) {
			wr(GENI_M_IRQ_CLEAR, status);
			return true;
		}
		sleep_us(50);
	}
	if (complaints++ < 8) {
		printf("fts: I2C write of %u bytes failed: status %#x\n", length, status);
	}
	geni_recover();
	return false;
}

/* One read; the controller releases the bus (and acknowledges the last byte) at the end. */
static bool
geni_i2c_read(uint8_t *data, uint32_t length)
{
	static unsigned complaints;
	uint32_t received = 0, status = 0;
	uint64_t deadline;

	if (length == 0) {
		return false;
	}
	if (!geni_wait_idle()) {
		geni_recover();
		return false;
	}
	wr(GENI_M_IRQ_CLEAR, 0xffffffff);
	wr(SE_I2C_RX_TRANS_LEN, length);
	wr(GENI_M_CMD0, I2C_OPCODE_READ | FTS_PARAMS);
	clock_interval_to_deadline(50, NSEC_PER_MSEC, &deadline);
	while (mach_absolute_time() < deadline) {
		uint32_t rx = rd(GENI_RX_FIFO_STATUS), words = rx & 0x01ffffff;

		while (words--) {
			uint32_t word = rd(GENI_RX_FIFO);

			for (int i = 0; i < 4 && received < length; i++, received++) {
				data[received] = (uint8_t)(word >> (8 * i));
			}
		}
		status = rd(GENI_M_IRQ_STATUS);
		if (status & M_ERRORS) {
			break;
		}
		if (status & M_CMD_DONE) {
			rx = rd(GENI_RX_FIFO_STATUS);
			words = rx & 0x01ffffff;
			while (words--) {
				uint32_t word = rd(GENI_RX_FIFO);

				for (int i = 0; i < 4 && received < length; i++, received++) {
					data[received] = (uint8_t)(word >> (8 * i));
				}
			}
			wr(GENI_M_IRQ_CLEAR, status);
			return received == length;
		}
		sleep_us(50);
	}
	if (complaints++ < 8) {
		printf("fts: I2C read of %u bytes failed: status %#x, received %u\n", length, status, received);
	}
	geni_recover();
	return false;
}

/* ---- the STMicroelectronics chip -------------------------------------------------------------------- */

#define FTS_READ_INFO           0x80    /* eight bytes: version, firmware, configuration, chip id */
#define FTS_READ_ONE_EVENT      0x85
#define FTS_SLEEP_OUT           0x91
#define FTS_MS_MT_SENSE_ON      0x93
#define FTS_SYSTEM_RESET        0xa0
#define FTS_CLEAR_EVENT_STACK   0xa1

#define FTS_EVENT_SIZE          8
#define FTS_EVENT_MAX           32
#define FTS_MAX_FINGERS         10

#define FTS_EV_ENTER            0x03
#define FTS_EV_LEAVE            0x04
#define FTS_EV_MOTION           0x05

#define FTS_MASK_EVENT_ID       0x0f
#define FTS_MASK_TOUCH_ID       0xf0
#define FTS_MASK_X_MSB          0x0f

static uint32_t max_x = 1080, max_y = 2248;
static uint32_t flip;

/* A command is one byte written to the chip's command register. */
static bool
fts_command(uint8_t command)
{
	return geni_i2c_write(&command, 1, false);
}

/* Commands that return data: the command byte, then the answer in the same transaction. */
static bool
fts_read(uint8_t command, uint8_t *data, uint32_t length)
{
	return geni_i2c_write(&command, 1, true) && geni_i2c_read(data, length);
}

static void
fts_reset(void)
{
	gpio_set(PIN_RESET, false);
	sleep_ms(10);
	gpio_set(PIN_RESET, true);
	sleep_ms(120);
}

/* Is the controller there and running its firmware? */
static bool
fts_probe(void)
{
	uint8_t info[8];
	bool blank = true;

	if (!fts_read(FTS_READ_INFO, info, sizeof(info))) {
		return false;
	}
	for (unsigned i = 0; i < sizeof(info); i++) {
		blank &= info[i] == 0 || info[i] == 0xff;
	}
	if (blank) {
		printf("fts: the chip answers with nothing (%02x %02x %02x %02x %02x %02x %02x %02x)\n", info[0], info[1],
		    info[2], info[3], info[4], info[5], info[6], info[7]);
		return false;
	}
	printf("fts: chip version %#x, firmware %#x.%#x, configuration %#x.%#x, chip id %#x\n", info[0],
	    info[2], info[3], info[4], info[5], (unsigned)info[6] << 8 | info[7]);
	return true;
}

static bool
bring_up(void)
{
	pins_init();
	if (!clocks_on()) {
		return false;
	}
	if (!geni_i2c_init()) {
		return false;
	}
	fts_reset();
	for (int attempt = 0; attempt < 3; attempt++) {
		if (fts_probe()) {
			break;
		}
		sleep_ms(50);
		if (attempt == 2) {
			return false;
		}
		fts_reset();
	}
	/* wake the controller and turn its multi-touch sensing on, as stmfts_power_on() and open() do */
	fts_command(FTS_SYSTEM_RESET);
	sleep_ms(50);
	fts_command(FTS_SLEEP_OUT);
	sleep_ms(100);
	fts_command(FTS_MS_MT_SENSE_ON);
	sleep_ms(50);
	fts_command(FTS_CLEAR_EVENT_STACK);
	sleep_ms(20);
	printf("fts: touch screen ready; interrupt line %s\n", gpio_get(PIN_INT) ? "high" : "low");
	return true;
}

/* ---- reports ---------------------------------------------------------------------------------------- */

static struct {
	bool down, seen;
	uint32_t x, y;
} contact[FTS_MAX_FINGERS];

static struct {
	uint32_t polls, events, touches, bad, last_x, last_y;
} seen;

static void
fts_contact_event(uint32_t id, uint32_t event, uint32_t x, uint32_t y)
{
	if (id >= FTS_MAX_FINGERS) {
		return;
	}
	if (event == FTS_EV_LEAVE) {
		contact[id].seen = true;
		if (contact[id].down) {
			contact[id].down = false;
			hakios_touch_post(HAKIOS_TOUCH_UP, id, contact[id].x, contact[id].y);
		}
		return;
	}
	if (max_x == 0 || max_y == 0) {
		return;
	}
	if (x >= max_x) {
		x = max_x - 1;
	}
	if (y >= max_y) {
		y = max_y - 1;
	}
	if (flip & 1) {
		x = max_x - 1 - x;
	}
	if (flip & 2) {
		y = max_y - 1 - y;
	}
	x = (uint32_t)((uint64_t)x * 65535 / (max_x - 1));
	y = (uint32_t)((uint64_t)y * 65535 / (max_y - 1));
	seen.touches++;
	seen.last_x = x;
	seen.last_y = y;
	contact[id].seen = true;
	if (!contact[id].down) {
		contact[id].down = true;
		hakios_touch_post(HAKIOS_TOUCH_DOWN, id, x, y);
	} else if (contact[id].x != x || contact[id].y != y) {
		hakios_touch_post(HAKIOS_TOUCH_MOVE, id, x, y);
	}
	contact[id].x = x;
	contact[id].y = y;
}

static void
fts_poll(void)
{
	static unsigned complaints;

	seen.polls++;
	for (uint32_t i = 0; i < FTS_MAX_FINGERS; i++) {
		contact[i].seen = false;
	}
	for (unsigned n = 0; n < FTS_EVENT_MAX; n++) {
		uint8_t event[FTS_EVENT_SIZE];
		uint8_t kind;

		if (!fts_read(FTS_READ_ONE_EVENT, event, sizeof(event))) {
			seen.bad++;
			if (complaints++ < 8) {
				printf("fts: cannot read the event stack\n");
			}
			break;
		}
		if (event[0] == 0) {
			break;          /* no more events */
		}
		seen.events++;
		kind = event[0] & FTS_MASK_EVENT_ID;
		if (kind == FTS_EV_ENTER || kind == FTS_EV_MOTION || kind == FTS_EV_LEAVE) {
			uint32_t id = (event[0] & FTS_MASK_TOUCH_ID) >> 4;
			uint32_t x = event[1] | ((event[2] & FTS_MASK_X_MSB) << 8);
			uint32_t y = (event[2] >> 4) | (event[3] << 4);

			fts_contact_event(id, kind, x, y);
		}
		/* 0x02 (a touch detected), 0x10/0x11 (the controller is ready) and status events need nothing */
	}
	/* a finger that was not in this batch has lifted */
	for (uint32_t id = 0; id < FTS_MAX_FINGERS; id++) {
		if (contact[id].down && !contact[id].seen) {
			contact[id].down = false;
			hakios_touch_post(HAKIOS_TOUCH_UP, id, contact[id].x, contact[id].y);
		}
	}
}

/* ---- the thread -------------------------------------------------------------------------------------- */

static void
fts_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint32_t poll_us = 8000, latency = 0;
	unsigned idle = 0, idle_looks;

	PE_parse_boot_argn("touch_poll_us", &poll_us, sizeof(poll_us));
	PE_parse_boot_argn("touch_flip", &flip, sizeof(flip));
	PE_parse_boot_argn("touch_max_x", &max_x, sizeof(max_x));
	PE_parse_boot_argn("touch_max_y", &max_y, sizeof(max_y));
	PE_parse_boot_argn("touch_lat", &latency, sizeof(latency));
	if (poll_us < 250 || poll_us > 100000) {
		poll_us = 8000;
	}
	if (max_x < 100 || max_y < 100) {
		max_x = 1080;
		max_y = 2248;
	}
	idle_looks = 80000 / poll_us;   /* a look every 80 ms even if the line is missed */

	tlmm = ml_io_map(TLMM_PHYS, TLMM_SIZE);
	gcc = ml_io_map(GCC_PHYS, GCC_SIZE);
	se = ml_io_map(SE_PHYS, SE_SIZE);

	if (!bring_up()) {
		printf("fts: no touch screen at I2C address %#x\n", FTS_ADDRESS);
		thread_terminate(current_thread());
		return;
	}
	for (unsigned tick = 1;; tick++) {
		uint64_t looked = 0;

		if (latency) {
			uint64_t ns;

			absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
			looked = ns / 1000;
		}
		if (!gpio_get(PIN_INT)) {
			idle = 0;
			fts_poll();
			if (latency) {
				uint64_t ns;

				absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
				hakios_tl("tl: f %llu %llu\n", looked, ns / 1000);
			}
		} else if (++idle >= idle_looks) {
			idle = 0;
			fts_poll();
		}
		if (tick % (60 * (1000000 / poll_us)) == 0) {
			printf("fts: so far %u events in %u looks (%u bad), %u finger positions, the last at %u, %u; "
			    "interrupt line %s\n", seen.events, seen.polls, seen.bad, seen.touches, seen.last_x, seen.last_y,
			    gpio_get(PIN_INT) ? "high" : "low");
		}
		sleep_us(poll_us);
	}
}

void
hakios_fts_init(void)
{
	/* 2 is this phone's controller; 1 stays the Goodix/FocalTech of redwood */
	uint32_t boot_arg = 0, dipper = 0;
	thread_t thread;

	if (!PE_parse_boot_argn("touchscreen", &boot_arg, sizeof(boot_arg)) || boot_arg != 2) {
		/* no explicit choice: the loader's "dipper=1" says this is the Mi 8 */
		if (!PE_parse_boot_argn("dipper", &dipper, sizeof(dipper)) || !dipper) {
			return;
		}
	}
	if (kernel_thread_start(fts_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}
