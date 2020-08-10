#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/serial_reg.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/debugfs.h>

#define SERIAL_BUFSIZE	16


struct serial_dev {
	struct miscdevice miscdev;
	void __iomem *regs;
	int irq;
	char serial_buf[SERIAL_BUFSIZE];
	int serial_buf_rd;
	int serial_buf_wr;
	wait_queue_head_t serial_wait;
	spinlock_t lock;
	int sent_counter_debug;
};

/*
static void dump_buf(struct serial_dev *dev)
{
	int i;

	for (i = 0; i < SERIAL_BUFSIZE; i++) {
		pr_info("!!! pos = %d; byte = %d; char = %c\n", i, dev->serial_buf[i], dev->serial_buf[i]);
	}
}
*/

static unsigned int reg_read(struct serial_dev *dev, int off)
{
	return readl(dev->regs + off * 4);
}

static void reg_write(struct serial_dev *dev, int val, int off)
{
	writel(val, dev->regs + off * 4);
}

static void send_byte(struct serial_dev *dev, char byte)
{
	while(!(reg_read(dev, UART_LSR) & UART_LSR_THRE)) {
	    cpu_relax();
	}
	reg_write(dev, byte, UART_TX);
}

ssize_t serial_read(struct file *file, char __user *buf, size_t len, loff_t *ppos)
{
	char byte;
	struct serial_dev *dev;
	int ret;
	unsigned long flags;


	dev = container_of(file->private_data, struct serial_dev, miscdev);

	ret = wait_event_interruptible(dev->serial_wait, dev->serial_buf_rd != dev->serial_buf_wr);
	if (ret)
		return ret;

	spin_lock_irqsave(&dev->lock, flags);

	byte = dev->serial_buf[dev->serial_buf_rd++];
	if (dev->serial_buf_rd == SERIAL_BUFSIZE)
		dev->serial_buf_rd = 0;

	spin_unlock_irqrestore(&dev->lock, flags);

	if (put_user(byte, buf))
		return -EFAULT;

	return 1;
}

ssize_t serial_write(struct file *file, const char __user *buf, size_t len, loff_t *ppos)
{
	char byte;
	struct serial_dev *dev;
	unsigned long count = 0;
	unsigned long flags;

	dev = container_of(file->private_data, struct serial_dev, miscdev);

	while (count < len) {
		if (get_user(byte, buf + count++))
			return -EFAULT;

		pr_debug("Byte to send: %x\n", byte);

		spin_lock_irqsave(&dev->lock, flags);

		send_byte(dev, byte);
		dev->sent_counter_debug++;
		if (byte == '\n')
			send_byte(dev, '\r');

		spin_unlock_irqrestore(&dev->lock, flags);
	}

	return count;
}

static const struct file_operations serial_fops = {
	.owner = THIS_MODULE,
	.read = serial_read,
	.write = serial_write,
};

static irqreturn_t serial_irq_handler(int irq, void *dev_id)
{
	char byte;
	struct serial_dev *dev = dev_id;

	spin_lock(&dev->lock);

	byte = reg_read(dev, UART_RX);

	dev->serial_buf[dev->serial_buf_wr++] = byte;
	if (dev->serial_buf_wr == SERIAL_BUFSIZE)
		dev->serial_buf_wr = 0;

	spin_unlock(&dev->lock);

	pr_debug("Get byte: %x\n", byte);

	wake_up(&dev->serial_wait);

	return IRQ_HANDLED;
}

static int serial_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct serial_dev *dev;
	unsigned int baud_divisor, uartclk;
	char *name;
	int ret;

	dev = devm_kzalloc(&pdev->dev, sizeof(struct serial_dev), GFP_KERNEL);
	if (!dev) {
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Can't get resource\n");
		return -EINVAL;
	}

	dev->regs = devm_ioremap_resource(&pdev->dev, res);
	if (!dev->regs) {
		dev_err(&pdev->dev, "Can't remap registers\n");
		return -ENOMEM;
	}

	dev->irq = platform_get_irq(pdev, 0);
	if (!dev->irq) {
		dev_err(&pdev->dev, "Can't get irq\n");
		return -ENOMEM;
	}
	pr_info("dev: %s, irq number from DT: %x", dev_name(&pdev->dev), dev->irq);

	ret = devm_request_irq(&pdev->dev, dev->irq, serial_irq_handler,
			       0, dev_name(&pdev->dev), dev);
	if (ret) {
		dev_err(&pdev->dev, "Interrupt request failed\n");
		return -ENOMEM;
	}

	init_waitqueue_head(&dev->serial_wait);

	spin_lock_init(&dev->lock);

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	/* Configure the baud rate to 115200 */
	of_property_read_u32(pdev->dev.of_node, "clock-frequency", &uartclk);
	baud_divisor = uartclk / 16 / 115200;
	reg_write(dev, 0x07, UART_OMAP_MDR1);
	reg_write(dev, 0x00, UART_LCR);
	reg_write(dev, UART_LCR_DLAB, UART_LCR);
	reg_write(dev, baud_divisor & 0xff, UART_DLL);
	reg_write(dev, (baud_divisor >> 8) & 0xff, UART_DLM);
	reg_write(dev, UART_LCR_WLEN8, UART_LCR);

	/* Enable interrupts */
	reg_write(dev, UART_IER_RDI, UART_IER);

	/* Soft reset */
	reg_write(dev, UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT, UART_FCR);
	reg_write(dev, 0x00, UART_OMAP_MDR1);

	dev->miscdev.minor = MISC_DYNAMIC_MINOR;
	name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "serial-%x", res->start);
	if (!name) {
		dev_err(&pdev->dev, "Cannot allocate buffer for misc name\n");
		return -ENOMEM;
	}
	dev->miscdev.name = name;
	dev->miscdev.fops = &serial_fops;

	ret = misc_register(&dev->miscdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register miscdevice: %d\n", ret);
		return ret;
	}

	name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "serial-sent_counter-%x", res->start);
	if (!name) {
		dev_err(&pdev->dev, "Cannot allocate buffer for debugfs file name\n");
		return -ENOMEM;
	}
	debugfs_create_u32(name, 0444, NULL, &dev->sent_counter_debug);

	return 0;
}

static int serial_remove(struct platform_device *pdev)
{
	struct serial_dev *dev;

	dev = platform_get_drvdata(pdev);

	misc_deregister(&dev->miscdev);

	pm_runtime_disable(&pdev->dev);

        return 0;
}

static const struct of_device_id serial_of_match[] = {
	{ .compatible = "bootlin,serial" },
	{ },
};

static struct platform_driver serial_driver = {
        .driver = {
                .name = "serial",
                .owner = THIS_MODULE,
		.of_match_table = serial_of_match,
        },
        .probe = serial_probe,
        .remove = serial_remove,
};

module_platform_driver(serial_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Max Kokhan");
