/*
 * sample.c - The simplest loadable kernel module.
 * Intended as a template for development of more
 * meaningful kernel modules.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <linux/semaphore.h>
#include <linux/jiffies.h>

/*
 * Definition of the GPIOs we want to use
*/

#define LED		129		// User led, PI.1
#define	ECHO	130		// User button, PI.11
#define TRIG 	131

#define PERIOD_TRIG	60

#define INVALID     -1
#define INIT_DRIVER 	0
#define BLINK_PERIOD	20

/*
 * Device major number
 */
static uint module_major = 166;

/*
 * Device name
 */
static char * module_name = "sample";

/*
 * Device access lock. Only one process can access the driver at a time
 */
static int sample_lock = 0;

/*
* Device variables
*/
struct timeval before, after, led1, led2;
int pulse = 0;
int led_period;
int tmp_period = 1;
int mode = INVALID;
int led_value = 1;
int time_led = 0;
//struct semaphore semaph;
struct semaphore sem_work;


/*
 * Declare the workqueue for the trigger
 */
static struct workqueue_struct *my_wq_trig;

typedef struct {
	struct work_struct my_work_trig;
} my_work_t_trig;

static my_work_t_trig work_trig;

/*
 * Declare the workqueue for the LED
 */
static struct workqueue_struct *my_wq_led;	/*this is the work queue*/

typedef struct {
	struct delayed_work my_work_led;				/*this is the work*/
} my_work_t_led;

static my_work_t_led work_led;

/*
 * Declare the workqueue for the blink
 */
static struct workqueue_struct *my_wq_blink;

typedef struct {
	struct work_struct my_work_blink;
} my_work_t_blink;

static my_work_t_blink work_blink;


/*
 * Work function for the blinking
 */
static void my_wq_function_blink( struct work_struct *work )
{

	my_work_t_blink *my_work;
	my_work = (my_work_t_blink *)work;
	sema_init(&sem_work, 0);
	
	while(1){
		
			down(&sem_work);
			queue_delayed_work (my_wq_led, (struct delayed_work *)&work_led, msecs_to_jiffies(led_period));
			flush_workqueue(my_wq_led);
	}
}

/*
 * Work function for the trigger
 */
static void my_wq_function_trig( struct work_struct *work )
{
	//sema_init(&semaph, 0);
	my_work_t_trig *my_work;
	my_work = (my_work_t_trig *)work;

	gpio_set_value(TRIG, 1);
	msleep(PERIOD_TRIG);
	gpio_set_value(TRIG, 0);
	
	/*while(1){

		gpio_set_value( TRIG, 1 );
		msleep( PERIOD_TRIG );
		gpio_set_value( TRIG, 0 );
		//msleep( PERIOD_TRIG );
		down(&semaph);
	}*/
	
}

/*
 * Work function for the LED
 */
static void my_wq_function_led( struct delayed_work *work )
{
	my_work_t_led *my_work;
	my_work = (my_work_t_led *)work;

	led_value = !led_value;
	gpio_set_value(LED, led_value);
	
	/*	TO VERIFY THE TIME OF THE PERIOD
			if(led_value == 1) do_gettimeofday(&led1);
			else{
			do_gettimeofday(&led2);
			time_led = ((led2.tv_sec- led1.tv_sec)*1000000)+(led2.tv_usec- led1.tv_usec);
			printk (KERN_INFO "time_led = %d\n", time_led);
			time_led = 0;
			}
	*/
	up(&sem_work);	
}

/*
 * Button interrupt handler
*/

static irq_handler_t echo_handler( unsigned int irq, struct pt_regs *regs )
{
	if ( gpio_get_value( ECHO ) == 1){
			do_gettimeofday(&before);
	}
	else{
			do_gettimeofday(&after);
			pulse = ((after.tv_sec- before.tv_sec)*1000000)+(after.tv_usec- before.tv_usec);
			queue_work( my_wq_trig, (struct work_struct *)&work_trig );
	}
	#ifdef SAMPLE_DEBUG
	printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif
	return (irq_handler_t)IRQ_HANDLED;
}

/*
 * Device open
 */
static int sample_open(struct inode *inode, struct file *file)
{
	int ret = 0;

	/*
	 * One process at a time
	 */
	if (sample_lock > 0) 
	{
		ret = -EBUSY;
	}
	else
	{
		sample_lock++;

		/*
 	 	* Increment the module use counter
 	 	*/
		try_module_get(THIS_MODULE);

		#ifdef SAMPLE_DEBUG 
			printk( KERN_INFO "%s: %s\n", module_name, __func__ ); 
		#endif
	}

	return( ret );
}


/*
 * Device close
 */
static int sample_release(struct inode *inode, struct file *file)
{
	/*
 	 * Release device
 	 */
	sample_lock = 0;

	/*
 	 * Decrement module use counter
 	 */
	module_put(THIS_MODULE);

	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif

	return( 0 );
}


/* 
 * Device read
 */
 
static ssize_t sample_read(struct file *filp, char *buffer,
			 size_t length, loff_t * offset)
{
	int ret = 1;

	memcpy(buffer, &pulse, sizeof(buffer));
	pulse = 0;
	//up(&semaph);
	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif

	return( ret );
}


/* 
 * Device write
 */
static ssize_t sample_write(struct file *filp, const char *buffer,
			  size_t length, loff_t * offset)
{
	int ret = 0;
	 
	
	switch( mode )
	{
		case INIT_DRIVER:
			queue_work( my_wq_trig, (struct work_struct *)&work_trig );
			memcpy(&led_period, buffer, sizeof(led_period));
			queue_work( my_wq_blink, (struct work_struct *)&work_blink );
			ret = 1;
			break;
		
		case BLINK_PERIOD:
			memcpy(&led_period, buffer, sizeof(led_period));
			
			if (tmp_period != led_period){
				
				if (led_period != 0){
						
						if(tmp_period != 0){
						cancel_delayed_work((struct delayed_work *)&work_led);
						flush_delayed_work((struct delayed_work *)&work_led);
						}

						led_value = !led_value;
						gpio_set_value(LED, led_value);
						up(&sem_work);
					}
				else{
						cancel_delayed_work((struct delayed_work *)&work_led);
						flush_delayed_work((struct delayed_work *)&work_led);
						gpio_set_value(LED, 1);
					}
					
			}
			tmp_period = led_period;
			ret = 2;
			break;
			
		case INVALID:
			ret = 0;
			break;
	}

	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif

	return( ret );
}


/*
* Device ioctl
*/
static ssize_t sample_ioctl(struct inode *inode, struct file *filep, 
			    const unsigned int cmd, const unsigned long arg)
{
	int ret = 0;

	switch( cmd )
	{
		case INIT_DRIVER:
			mode = INIT_DRIVER;	
			break;
			
		case BLINK_PERIOD:
			mode = BLINK_PERIOD;
			break;
			
		default:
			mode = INVALID;
			break;
	}

	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif

	return( ret );
}

/*
 * Device operations
 */
static struct file_operations sample_fops = {
	.read = sample_read,
	.write = sample_write,
	.open = sample_open,
	.release = sample_release,
	.ioctl = sample_ioctl
};


static int __init sample_init_module(void)
{
	/*
 	 * Register device
 	 */
	int	ret;

	ret = register_chrdev(module_major, module_name, &sample_fops);
	if (ret < 0) {
		printk(KERN_INFO "%s: registering device %s with major %d failed with %d\n",
		       __func__, module_name, module_major, module_major );
		return( ret );
	}
	else
	{
		printk(KERN_INFO "%s: registering device %s with major %d\n",
		       __func__, module_name, module_major );

		/*
 		 * Reserve gpios ECHO (as input), TRIG and LED (as output, with default output value set to 0)
		*/
		if( gpio_request( ECHO, module_name ) )	// Check if ECHO is available
		{
			printk( KERN_INFO "%s: %s unable to get BTN gpio\n", module_name, __func__ );
			ret = -EBUSY;
			return( ret );
		}
		
		if( gpio_request( TRIG, module_name ) )	// Check if TRIG is available
		{
			printk( KERN_INFO "%s: %s unable to get BTN gpio\n", module_name, __func__ );
			ret = -EBUSY;
			return( ret );
		}

		if( gpio_request( LED, module_name ) )	// Check if LED is available
		{
			printk( KERN_INFO "%s: %s unable to get LED gpio\n", module_name, __func__ );
			ret = -EBUSY;
			return( ret );
		}

		if( gpio_direction_input( ECHO ) < 0 )	// Set ECHO gpio as input
		{
			printk( KERN_INFO "%s: %s unable to set BTN gpio as input\n", module_name, __func__ );
			ret = -EBUSY;
			return( ret );
		}

		if( gpio_direction_output( TRIG, 0 ) < 0 )	// Set TRIG gpio as output with default value 0
		{
			printk( KERN_INFO "%s: %s unable to set LED gpio as output\n", module_name, __func__ );
			ret = -EBUSY;
			return( ret );
		}
		
		if( gpio_direction_output( LED, 0 ) < 0 )	// Set LED gpio as output with default value 0
		{
			printk( KERN_INFO "%s: %s unable to set LED gpio as output\n", module_name, __func__ );
			ret = -EBUSY;
			return( ret );
		}

		if( request_irq( gpio_to_irq( ECHO ), 
                                 (irq_handler_t) echo_handler,
				  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING ,
				  module_name,
				  NULL ) < 0 )
		{
			printk( KERN_INFO "%s: %s unable to register gpio irq for BTN\n", module_name, __func__ );
			ret = -EBUSY;
			return( ret );
		}
		
		my_wq_trig = create_workqueue( "my_queue_trig" );
		if( my_wq_trig )
		{
			INIT_WORK( (struct work_struct *)&work_trig, my_wq_function_trig );
		}
		
		my_wq_blink = create_workqueue( "my_queue_blink" );
		if( my_wq_blink )
		{
			INIT_WORK( (struct work_struct *)&work_blink, my_wq_function_blink );
		}
		
		my_wq_led = create_workqueue( "my_queue_led" );
		if( my_wq_led )
		{
			INIT_DELAYED_WORK( (struct delayed_work *)&work_led, my_wq_function_led );
		}
	}
	
	return( ret );
}


static void __exit sample_cleanup_module(void)
{
	/*
	 * Free irq
	 */
	free_irq( gpio_to_irq( ECHO ), NULL );

	/*
	 * Release the gpios
	 */
	gpio_free( ECHO );
	gpio_free( TRIG );
	gpio_free( LED );

	/*
	 * Unregister device
	 */
	unregister_chrdev(module_major, module_name);

	printk(KERN_INFO "%s: unregistering %s done\n", __func__, module_name );
}

module_init(sample_init_module);
module_exit(sample_cleanup_module);
destroy_workqueue(my_wq_led);
destroy_workqueue(my_wq_trig);
destroy_workqueue(my_wq_blink);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alessandro Di Chiara, alexdichiara7@gmail.com");
MODULE_DESCRIPTION("Device Driver HCSR04");
