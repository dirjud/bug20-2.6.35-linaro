/*
 * 	bmi_bugduino.c
 *
 * 	BMI von Hippel device driver basic functionality
 *
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 *	Include files
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/poll.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <mach/hardware.h>

#include <linux/i2c.h>

#include <linux/bmi.h>
#include <linux/bmi/bmi-slot.h>
#include <linux/bmi/bmi_bugduino.h>

#define BMIBUGDUINO_VERSION	"1.0"

/*
 * 	Global variables
 */

static struct i2c_board_info iox_info = {
  I2C_BOARD_INFO("BUGDUINO_IOX", BMI_IOX_I2C_ADDRESS),
};

// private device structure
struct bmi_bugduino
{
  struct bmi_device	*bdev;			// BMI device
  struct cdev		cdev;			// control device
  struct device	*class_dev;		// control class device
  int			open_flag;		// single open flag
  char			int_name[20];		// interrupt name
  struct i2c_client *iox;
  struct i2c_client *i2c_dev;
  unsigned char curr_i2c_addr;	//Current i2c address
  struct spi_device *spi;       // SPI device
  struct spi_board_info bugduino_spi_info;
  char                  rbuf[BUF_MAX_SIZE];     // SPI read buffer
  char                  wbuf[BUF_MAX_SIZE];     // SPI write buffer
  ssize_t ints;				// Number of interrupts
  wait_queue_head_t  intq;		// Wait queue for interupts
};

static struct bmi_bugduino bmi_bugduino[4];	// per slot device structure
static int major;		// control device major

/*
 * 	BMI set up
 */

// BMI device ID table
static struct bmi_device_id bmi_bugduino_tbl[] = 
{ 
	{ 
		.match_flags = BMI_DEVICE_ID_MATCH_VENDOR | BMI_DEVICE_ID_MATCH_PRODUCT, 
		.vendor   = BMI_VENDOR_SEEED_STUDIO, 
		.product  = BMI_PRODUCT_BUGDUINO, 
		.revision = BMI_ANY, 
	}, 
	{ 0, },	  /* terminate list */
};
MODULE_DEVICE_TABLE(bmi, bmi_bugduino_tbl);

int	bmi_bugduino_probe (struct bmi_device *bdev);
void	bmi_bugduino_remove (struct bmi_device *bdev);
int	bmi_bugduino_resume (struct device *dev);
int	bmi_bugduino_suspend (struct device *dev);

static struct dev_pm_ops bmi_bugduino_pm =
{
	.resume = bmi_bugduino_resume,
	.suspend = bmi_bugduino_suspend,
};

// BMI driver structure
static struct bmi_driver bmi_bugduino_driver = 
{
	.name = "bmi_bugduino", 
	.id_table = bmi_bugduino_tbl, 
	.probe   = bmi_bugduino_probe, 
	.remove  = bmi_bugduino_remove, 
	.pm  = &bmi_bugduino_pm,
};

/*
 * 	I2C set up
 */

static void chk_i2c_dev (struct bmi_bugduino * bugduino, struct i2c_xfer * xfer){
	if (bugduino->curr_i2c_addr != xfer->addr){
	   if (bugduino->i2c_dev != NULL)
	   	i2c_unregister_device(bugduino->i2c_dev);
	   struct i2c_board_info i2c_info = {
	      I2C_BOARD_INFO( "bugduino", xfer->addr ),
	   };
	   bugduino->i2c_dev = i2c_new_device( bugduino->bdev->slot->adap, &i2c_info );
	   bugduino->curr_i2c_addr = xfer->addr;
	}
}

static int ReadBytes_I2C (struct i2c_client *client, unsigned char offset, char * data, int len){
	int ret=0;
	
	ret = i2c_master_send(client, &offset, 1);
	if (ret == 1)
		ret = i2c_master_recv(client, data, len);
	if (ret < 0)
		printk (KERN_ERR "ReadByte_I2C() - i2c_transfer() failed... %d\n",ret);
	return ret;
}

static int WriteBytes_I2C (struct i2c_client *client, unsigned char offset, char * data, int len){
	int ret = 0;
	int i;
	char msg[BUF_MAX_SIZE];
	msg[0] = offset;

	for (i=0;i<len;i++)
		msg[i+1] = data[i];
	ret = i2c_master_send(client, msg, len+1);
	
	if (ret < 0)
	  printk (KERN_ERR "WriteByte_IOX() - i2c_transfer() failed...%d\n",ret);

	return ret;
}

// IOX
// read byte from I2C IO expander
static int ReadByte_IOX (struct i2c_client *client, unsigned char offset, unsigned char *data)
{
	int	ret = 0;	

	ret = i2c_master_send(client, &offset, 1);
	if (ret == 1)
	  ret = i2c_master_recv(client, data, 1);
	if (ret < 0)
	  printk (KERN_ERR "ReadByte_IOX() - i2c_transfer() failed...%d\n",ret);
	return ret;
}

// write byte to I2C IO expander
static int WriteByte_IOX (struct i2c_client *client, unsigned char offset, unsigned char data)
{
	int	ret = 0;
	unsigned char msg[2];
	
     	msg[0] = offset;
	msg[1] = data;
	ret = i2c_master_send(client, msg, sizeof(msg));
	
	if (ret < 0)
	  printk (KERN_ERR "WriteByte_IOX() - i2c_transfer() failed...%d\n",ret);

	return ret;
}

/*
 *	control device operations
 */

// open
int cntl_open(struct inode *inode, struct file *file)
{	
	struct bmi_bugduino *bugduino;

	bugduino = container_of (inode->i_cdev, struct bmi_bugduino, cdev);

	// Enforce single-open behavior

	if (bugduino->open_flag) {
		return -EBUSY; 
	}
	bugduino->open_flag = 1;

	// Save bugduino_dev pointer for later.

	file->private_data = bugduino;
	return 0;

}

// release
int cntl_release(struct inode *inode, struct file *file)
{	
	struct bmi_bugduino *bugduino;

	bugduino = (struct bmi_bugduino *)(file->private_data);
	wake_up_interruptible(&(bugduino->intq));
	bugduino->open_flag = 0;
	return 0;
}

// ioctl
int cntl_ioctl(struct inode *inode, struct file *file, unsigned int cmd, 
		   unsigned long arg)
{	
	struct i2c_adapter *adap;
	unsigned char iox_data;
	//	unsigned char buf[4];
	char msg[BUF_MAX_SIZE];
	int ret;

	struct bmi_bugduino *bugduino;
	int slot;

	bugduino = (struct bmi_bugduino *)(file->private_data);

	// error if bugduino not present
	if(bugduino->bdev == 0)
		return -ENODEV;
	
	slot = bugduino->bdev->slot->slotnum;
	adap = bugduino->bdev->slot->adap;

	// ioctl's
	switch (cmd) {

	case BMI_BUGDUINO_RLEDOFF:
	  bmi_slot_gpio_set_value (slot, BUGDUINO_GPIO_RED_LED, 1); // Red LED=OFF 
		break;

	case BMI_BUGDUINO_RLEDON:
	  bmi_slot_gpio_set_value (slot, BUGDUINO_GPIO_RED_LED, 0); // Red LED=ON 
		break;

	case BMI_BUGDUINO_GLEDOFF:
	  bmi_slot_gpio_set_value (slot, BUGDUINO_GPIO_GREEN_LED, 1); // Green LED=OFF 
	  break;

	case BMI_BUGDUINO_GLEDON:
	  bmi_slot_gpio_set_value (slot, BUGDUINO_GPIO_GREEN_LED, 0); // Green LED=ON
	  break;

	case BMI_BUGDUINO_GETSTAT:
		{
			int read_data;
	
			if (ReadByte_IOX (bugduino->iox, IOX_INPUT_REG, &iox_data) < 0)
				return -ENODEV;			
			read_data = iox_data | (bmi_slot_gpio_get_all(slot) << 8);

			if (put_user (read_data, (int __user *) arg))
				return -EFAULT;
		}
		break;

	case BMI_BUGDUINO_RESET:
		bmi_slot_gpio_set_value( slot, BUGDUINO_GPIO_RESET_PIN, ( ( arg == 0 ) ? ( BUGDUINO_RESET_OFF ) : ( BUGDUINO_RESET_ON ) ) );
		break;

	case BMI_BUGDUINO_I2C_WRITE:
		{
			struct i2c_xfer xfer;
			ret = copy_from_user( &xfer, (struct i2c_xfer __user *)arg, sizeof( struct i2c_xfer) );
			if (ret != 0){
				printk( KERN_ERR "bmi_bugduino.c: Error copying i2c_xfer from user to kernel space. %i\n", ret);
				return ret;
			}
			memset(msg, 0, xfer.len);
			ret = copy_from_user( msg, xfer.data, xfer.len );
			if (ret != 0){
				printk( KERN_ERR "bmi_bugduino.c: Error copying i2c data from user to kernel space. %i\n", ret);
				return ret;
			}
			chk_i2c_dev (bugduino, &xfer);
			ret = WriteBytes_I2C (bugduino->i2c_dev, xfer.offset, msg, xfer.len);
			if (ret < 0)
				printk( KERN_ERR "bmi_bugduino.c: Error writing i2c bytes %i\n", ret);
			return ret;
			
		}
		break;

	case BMI_BUGDUINO_I2C_READ:
		{
			struct i2c_xfer xfer;
			ret = copy_from_user( &xfer, (struct i2c_xfer __user *)arg, sizeof( struct i2c_xfer) );
			if (ret != 0){
				printk( KERN_ERR "bmi_bugduino.c: Error copying i2c_xfer from user to kernel space. %i\n", ret);
				return ret;
			}
			memset(msg, 0, xfer.len);
			chk_i2c_dev (bugduino, &xfer);
			ret = ReadBytes_I2C (bugduino->i2c_dev, xfer.offset, msg, xfer.len);
			if (ret < 0)
				printk( KERN_ERR "bmi_bugduino.c: Error writing i2c bytes %i\n", ret);
			ret = copy_from_user( msg, xfer.data, xfer.len );
			if (ret != 0){
				printk( KERN_ERR "bmi_bugduino.c: Error copying i2c data from user to kernel space. %i\n", ret);
			}
			return ret;
		}
		break;

	case BMI_BUGDUINO_MKIOX_OUT:
		if ((arg < BUGDUINO_IOX_B0) || (arg > BUGDUINO_IOX_B7))
			return -EINVAL;
		{
			unsigned char read_data;
	
			if (ReadByte_IOX (bugduino->iox, IOX_CONTROL, &iox_data) < 0)
				return -ENODEV;
		
			read_data = iox_data & ~(0x1 << arg);

			if (WriteByte_IOX (bugduino->iox, IOX_CONTROL, read_data) < 0)
				return -ENODEV;
		}
		break;

	case BMI_BUGDUINO_MKIOX_IN:
		if ((arg < BUGDUINO_IOX_B0) || (arg > BUGDUINO_IOX_B7))
			return -EINVAL;
		{
			unsigned char read_data;
	
			if (ReadByte_IOX (bugduino->iox, IOX_CONTROL, &iox_data) < 0)
				return -ENODEV;
		
			read_data = iox_data & (0x1 << arg);

			if (WriteByte_IOX (bugduino->iox, IOX_CONTROL, read_data) < 0)
				return -ENODEV;
		}
		break;

	case BMI_BUGDUINO_SETIOX:
		if ((arg < BUGDUINO_IOX_B0) || (arg > BUGDUINO_IOX_B7))
			return -EINVAL;
		{
			unsigned char read_data;
	
			if (ReadByte_IOX (bugduino->iox, IOX_OUTPUT_REG, &iox_data) < 0)
				return -ENODEV;
		
			read_data = iox_data | (0x1 << arg);

			if (WriteByte_IOX (bugduino->iox, IOX_OUTPUT_REG, read_data) < 0)
				return -ENODEV;
		}
		break;

	case BMI_BUGDUINO_CLRIOX:
		if ((arg < BUGDUINO_IOX_B0) || (arg > BUGDUINO_IOX_B7))
			return -EINVAL;
		{
			unsigned char read_data;
	
			if (ReadByte_IOX (bugduino->iox, IOX_OUTPUT_REG, &iox_data) < 0)
				return -ENODEV;
		
			read_data = iox_data & ~(0x1 << arg);

			if (WriteByte_IOX (bugduino->iox, IOX_OUTPUT_REG, read_data) < 0)
				return -ENODEV;
		}
		break;

	case BMI_BUGDUINO_SUSPEND:
		bugduino->bdev->dev.bus->pm->suspend(&bugduino->bdev->dev);
		break;
	case BMI_BUGDUINO_RESUME:
		bugduino->bdev->dev.bus->pm->resume(&bugduino->bdev->dev);
		break;
	default:
		return -ENOTTY;
	}

	return 0;
}

//cntl_read - will block until interrupt is recieved
ssize_t cntl_read (struct file *file, char * buff, size_t bufflen, loff_t * offset){
	ssize_t ret;
	//Right now, we only care about the file
	//Future implementations could use the buffer or offset for addl functionality
	struct bmi_bugduino *bugduino;

        bugduino = (struct bmi_bugduino *)(file->private_data);		//Retrieve the BUGDUINO struct

	if(bugduino->bdev == 0)					//Check to make sure it is opened
                return -ENODEV;

	if (bugduino->ints > 0){					//If interrupts have been recieved
		ret = bugduino->ints;
		bugduino->ints = 0;
		return ret;
	}

	//Prepare to sleep
	DEFINE_WAIT(wait);
	add_wait_queue(&(bugduino->intq), &wait);
	while (bugduino->ints == 0){				//Wake up when ints > 0
		prepare_to_wait(&(bugduino->intq), &wait, TASK_INTERRUPTIBLE);
		if (signal_pending(current))
			break;				//This should be more robust!
		schedule();
	}
	finish_wait(&(bugduino->intq), &wait);
	ret = bugduino->ints;
	bugduino->ints = 0;
	return ret;				//If returns 0, we were signal'd
						//OTW, we actually recieved an interrupt!
}

unsigned int cntl_poll(struct file *file, poll_table *wait){
	struct bmi_bugduino *bugduino;
	unsigned int mask = 0;
	bugduino = (struct bmi_bugduino *)(file->private_data);
	poll_wait(file, &(bugduino->intq), wait);
	if (bugduino->ints > 0)
		mask |= POLLIN | POLLRDNORM;
	return mask;
}

// control file operations
struct file_operations cntl_fops = {
	.owner = THIS_MODULE, 
	.ioctl = cntl_ioctl, 
	.open = cntl_open, 
	.release = cntl_release, 
	.read = cntl_read,
	.poll = cntl_poll,
};

/*
 *	PIM functions
 */

// interrupt handler
static irqreturn_t module_irq_handler(int irq, void *dummy)
{
	struct bmi_bugduino *bugduino = dummy;
	bugduino->ints++;
	wake_up_interruptible(&(bugduino->intq));
	return IRQ_HANDLED;
}

/*
 * 	BMI functions
 */

// probe - insert PIM
int bmi_bugduino_probe(struct bmi_device *bdev)
{
	int err;
	int slot;
	struct bmi_bugduino *bugduino;
       	struct i2c_adapter *adap;
	struct cdev *cdev;
	struct class *bmi_class;
	dev_t dev_id;
	int irq;
	unsigned long speed = 1000000;
        unsigned char mode = SPI_MODE_0;

	err = 0;
	slot = bdev->slot->slotnum;
      	adap = bdev->slot->adap;
	bugduino = &bmi_bugduino[slot];

	bugduino->bdev = 0;
	bugduino->open_flag = 0;
	bugduino->ints = 0;
	init_waitqueue_head(&(bugduino->intq));
	
	// Create 1 minor device
	cdev = &bugduino->cdev;
	cdev_init (cdev, &cntl_fops);

	dev_id = MKDEV(major, slot); 
	err = cdev_add (cdev, dev_id, 1);
	if (err) {
		return err;
	}

	// Create class device 
	bmi_class = bmi_get_class ();                            
	bugduino->class_dev = device_create (bmi_class, NULL, MKDEV (major, slot), NULL, "bmi_bugduino_m%i", slot);  
								     
	if (IS_ERR(bugduino->class_dev)) {                                
		printk (KERN_ERR "Unable to create "                  
		       "class_device for bmi_bugduino_m%i; errno = %ld\n",
		       slot, PTR_ERR(bugduino->class_dev));             
		bugduino->class_dev = NULL;                               
		cdev_del (&bugduino->cdev);
		return -ENODEV;
	}                                                            

	// bind driver and bmi_device 
	bugduino->bdev = bdev;
	

	printk (KERN_INFO "bmi_bugduino.c: probe slot %d\n", slot);
	bugduino->iox = i2c_new_device(bdev->slot->adap, &iox_info);
	if (bugduino->iox == NULL)
	  printk(KERN_ERR "IOX NULL...\n");
	
	// SPI
	strcpy(bugduino->bugduino_spi_info.modalias, "spidev");
	bugduino->bugduino_spi_info.max_speed_hz = speed;
	bugduino->bugduino_spi_info.bus_num = bdev->slot->spi_bus_num;
	bugduino->bugduino_spi_info.chip_select = bdev->slot->spi_cs;
	bugduino->bugduino_spi_info.mode = mode;

	bugduino->spi = spi_new_device(spi_busnum_to_master(bugduino->bugduino_spi_info.bus_num), &bugduino->bugduino_spi_info) ;
	if (!bugduino->spi)
	  printk(KERN_WARNING "BUGDUINO: spi_new_device failed\n");

	bmi_device_set_drvdata (bdev, bugduino);
	
	if (WriteByte_IOX(bugduino->iox, IOX_OUTPUT_REG, 0x7F) < 0) {  // outputs high
		goto err1;
	}

	if (WriteByte_IOX(bugduino->iox, IOX_CONTROL, 0x00) < 0) {     // IOX[7:0]=OUT
		goto err1;
	}

	mdelay(100);

	// Initialize GPIOs (turn LED's on)
	bmi_slot_gpio_direction_out (slot, RED_LED, 0);	// Red LED=ON
	bmi_slot_gpio_direction_out (slot, GREEN_LED, 0);	// Red LED=ON
	
	mdelay(200);
	
	// turn LED's off
	bmi_slot_gpio_set_value (slot, RED_LED, 1);
	bmi_slot_gpio_set_value (slot, GREEN_LED, 1);		// Red, Green LED=OFF 	
	bmi_slot_gpio_direction_out (slot, BUGDUINO_GPIO_RESET_PIN, 0);	// AVR Reset Pin

	if (WriteByte_IOX(bugduino->iox, IOX_OUTPUT_REG, 0x00) < 0) {  // IOX[7:0] low
	  printk (KERN_ERR "bmi_bugduino.c: probe() - write IOX failed\n");
	  //bmi_device_spi_cleanup(bdev);
	  goto err1;
	}
	

	// request PIM interrupt
	irq = bdev->slot->status_irq;
	sprintf (bugduino->int_name, "bmi_bugduino%d", slot);
	if (request_irq(irq, &module_irq_handler, IRQF_TRIGGER_FALLING, bugduino->int_name, bugduino)) {
		printk (KERN_ERR "bmi_bugduino.c: Can't allocate irq %d or find von Hippel in slot %d\n", 
			irq, slot); 
		//bmi_device_spi_cleanup(bdev);
		goto err1;

		//return -EBUSY;
	}

	return 0;

 err1:	
	bugduino->class_dev = NULL;                               
	cdev_del (&bugduino->cdev);
	device_destroy (bmi_class, MKDEV(major, slot));
	bmi_device_set_drvdata (bdev, 0);
	bugduino->bdev = 0;
	i2c_unregister_device(bugduino->iox);
	spi_unregister_device(bugduino->spi);
	return -ENODEV;
}

// remove PIM
void bmi_bugduino_remove(struct bmi_device *bdev)
{	
	int slot;
	struct bmi_bugduino *bugduino;
	struct class *bmi_class;
	int irq;
	int i;

	printk(KERN_INFO "bmi_bugduino: Module Removed...\n");
	slot = bdev->slot->slotnum;
	bugduino = &bmi_bugduino[slot];

	i2c_unregister_device(bugduino->iox);
	spi_unregister_device(bugduino->spi);
	if (bugduino->i2c_dev != NULL)
		i2c_unregister_device(bugduino->i2c_dev);

	irq = bdev->slot->status_irq;
	free_irq (irq, bugduino);

	for (i = 0; i < 4; i++)
	  bmi_slot_gpio_direction_in(slot, i);

	bmi_class = bmi_get_class ();
	device_destroy (bmi_class, MKDEV(major, slot));

	bugduino->class_dev = 0;

	cdev_del (&bugduino->cdev);

	// de-attach driver-specific struct from bmi_device structure 
	bmi_device_set_drvdata (bdev, 0);
	bugduino->bdev = 0;

	return;
}

/*
 *	PM routines
 */

int bmi_bugduino_resume(struct device *dev)
{
	struct bmi_device *bmi_dev;

	bmi_dev = to_bmi_device(dev);

	printk(KERN_INFO "bmi_bugduino: Resume..\n");
	bmi_slot_uart_enable(bmi_dev->slot->slotnum);
	bmi_slot_spi_enable(bmi_dev->slot->slotnum);
	return 0;
}

int bmi_bugduino_suspend(struct device *dev)
{
	struct bmi_device *bmi_dev;

	bmi_dev = to_bmi_device(dev);

	printk(KERN_INFO "bmi_bugduino: Suspend..\n");
	bmi_slot_uart_disable(bmi_dev->slot->slotnum);
	bmi_slot_spi_disable(bmi_dev->slot->slotnum);
	return 0;
}

/*
 *	module routines
 */

static void __exit bmi_bugduino_cleanup(void)
{
	dev_t dev_id;

	bmi_unregister_driver (&bmi_bugduino_driver);

	dev_id = MKDEV(major, 0);
	unregister_chrdev_region (dev_id, 4);
	return;
}

static int __init bmi_bugduino_init(void)
{
	dev_t	dev_id;
	int	retval;

	// alloc char driver with 4 minor numbers
	retval = alloc_chrdev_region (&dev_id, 0, 4, "BMI BUGDUINO Driver"); 
	if (retval) {
		return -ENODEV;
	}

	major = MAJOR(dev_id);
	retval = bmi_register_driver (&bmi_bugduino_driver);   
	if (retval) {
		unregister_chrdev_region(dev_id, 4);
		return -ENODEV;  
	}

	printk (KERN_INFO "bmi_bugduino.c: BMI_BUGDUINO Driver v%s \n", BMIBUGDUINO_VERSION);

	return 0;
}


module_init(bmi_bugduino_init);
module_exit(bmi_bugduino_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Buglabs Inc.");
MODULE_DESCRIPTION("BMI von Hippel device driver");
MODULE_SUPPORTED_DEVICE("bmi_bugduino_mX");

