#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/uaccess.h>  /*copy from ... copy to...*/
#include <asm/irq.h>
#include <asm/io.h> /*readl writel ioremap*/
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h> /*platform_driver_register*/
#include <linux/device.h> /*device_create class_create*/
#include <linux/slab.h> /*kzalloc kfree*/
#include <linux/of.h> /*struct device_node*/
#include <linux/types.h>
#include <linux/ioctl.h>

#define   DRV_DESCRIPTION   "fpga acc"
#define   DRV_VERSION       "0.1"

MODULE_DESCRIPTION(DRV_DESCRIPTION);
MODULE_AUTHOR("Peng Hu <hupeng19g@ict.ac.cn>");
MODULE_VERSION(DRV_VERSION);
MODULE_LICENSE("GPL");

#define FPGA_ACC_IOC_MAGIC  0xC0

//HuPeng: add ioctl for acc  
#define FPGA_ACC_IOC_REGISTER_SECURE_DEV \
  _IOR(FPGA_ACC_IOC_MAGIC, 0x00, struct fpga_acc_ioctl_sec_dev_register)

struct fpga_acc {
    dev_t devno;              /* 设备号 */
    int major;                /* 主设备号 */
    int minor;                /* 次设备号 */
    struct cdev cdev;         /* 字符设备 */
    struct class *class;      /* 类结构体 */
    struct device *device;    /* 设备 */
    struct device_node *nd;   /* 设备节点 */
    u64 mmio_base;
    u64 mmio_size;
};

static struct fpga_acc fpga_acc;

struct fpga_acc_ioctl_sec_dev_register{
  unsigned long eid;
  int devid;//point the index of sec_dev_drv
  char secure_drv_name[512];
};

/* the hardware info of peripheral device to register */
struct device_register_info
{
  dev_t devno;//设备唯一的设备号
  u64 mmio_base;
  size_t mmio_size;
};

static int fpga_acc_open (struct inode *node, struct file *filp)
{
	return 0;
}

static ssize_t fpga_acc_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
    return 0;
}

static ssize_t fpga_acc_write (struct file *filp, const char __user *buf, size_t size, loff_t *off)
{
	return 0; 
}

static int fpga_acc_release (struct inode *node, struct file *filp)
{
	return 0;
}

extern int keystone_device_register(unsigned long ueid, struct device_register_info dev_info, int* enclave_devid);

static long fpga_acc_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
  int ret = 0;
  //printk("fpga_acc_ioct\r\n");
  
  switch(cmd){
    case FPGA_ACC_IOC_REGISTER_SECURE_DEV:
    {
      const char *str;
      struct fpga_acc_ioctl_sec_dev_register dev_register_local_args; 
      if (copy_from_user(&dev_register_local_args, (void __user *) arg, sizeof(struct fpga_acc_ioctl_sec_dev_register)))
        return -EFAULT;
      
      /* 获取可信外设驱动名字 */
      ret = of_property_read_string(fpga_acc.nd, "trusted_drv_name",&str);
      if (ret < 0) {
         printk("[fpga_acc drv] does not has a trusted device driver\r\n");
         return -EFAULT;
      } else {
        strcpy(dev_register_local_args.secure_drv_name,str);
        printk("[fpga_acc drv] trusted device driver file path is:%s\r\n", dev_register_local_args.secure_drv_name);
      }
      
      struct file *filp  = NULL;
      filp = filp_open(dev_register_local_args.secure_drv_name, O_RDONLY, 0);
      if (IS_ERR(filp)) {
        printk("[fpga_acc drv] trusted device driver file(%s) dose not exist\r\n",dev_register_local_args.secure_drv_name);
        return -EFAULT;
      } 
      filp_close(filp, NULL);  

      struct device_register_info dev;
      dev.devno = fpga_acc.devno;
      dev.mmio_base = fpga_acc.mmio_base;
      dev.mmio_size = fpga_acc.mmio_size;
      
      printk("[fpga_acc drv] register trusted device to keystone-driver\r\n");
      int enclave_devid = -1;
      ret = keystone_device_register(dev_register_local_args.eid, dev, &enclave_devid);
	    if(ret)
        printk("[fpga_acc drv] failed to register trusted device\r\n");

      printk("[fpga_acc drv] device id assigned by keystone-driver is:#%d\r\n",enclave_devid);
      dev_register_local_args.devid = enclave_devid;
      
      ret = copy_to_user((void __user*) arg, &dev_register_local_args, sizeof(struct fpga_acc_ioctl_sec_dev_register));

      break;
    }
    default:
      printk("unkownd cmd...\n");
      return -ENOSYS;
  }
  return ret;
}

static struct file_operations fpga_acc_ops = {
	.owner = THIS_MODULE,
	.open  = fpga_acc_open,
  .read = fpga_acc_read,
	.write = fpga_acc_write,
  .unlocked_ioctl = fpga_acc_ioctl,
	.release = fpga_acc_release,
};

static int __init fpga_acc_init(void)
{
    //printk("[fpga_acc drv] init\r\n");
  
    int ret;
    /* 申请设备号 */
    fpga_acc.major = 0;
    if (fpga_acc.major){
      fpga_acc.devno = MKDEV(fpga_acc.major, fpga_acc.minor);
      ret = register_chrdev_region(fpga_acc.devno, 1, "fpga_acc");
    }
    else{
      ret = alloc_chrdev_region(&fpga_acc.devno, 0, 1, "fpga_acc"); /* get devno */
      fpga_acc.major = MAJOR(fpga_acc.devno); /* get major */
      fpga_acc.minor = MINOR(fpga_acc.devno);        
    }
    if (ret < 0){
      printk("[fpga_acc drv] can't get devno\r\n");
      goto fail_devno;
    }
    
    /* 注册字符设备 */
    cdev_init(&fpga_acc.cdev, &fpga_acc_ops);
    ret = cdev_add(&fpga_acc.cdev, fpga_acc.devno, 1);
    if(ret){
        printk("[fpga_acc drv] fail to register the cdev to the kernel\r\n");
        goto fail_cdev;
    }

    /* 自动创建设备节点 */
    fpga_acc.class = class_create(THIS_MODULE, "fpga_acc");
    if (IS_ERR(fpga_acc.class)) {
        ret = PTR_ERR(fpga_acc.class);
        goto fail_class;
    }

	  fpga_acc.device = device_create(fpga_acc.class, NULL, fpga_acc.devno, NULL, "fpga_acc"); // /dev/fpga_acc
    if (IS_ERR(fpga_acc.device)) {
        ret = PTR_ERR(fpga_acc.device);
        goto fail_device;
    }

    //printk("dts_fpga_acc loading!\r\n");
    
    /* 获取设备树的属性内容 */
    fpga_acc.nd = of_find_compatible_node(NULL, NULL, "ict,fpga_acc");
    if(fpga_acc.nd == NULL){
      printk("[fpga_acc drv] dts_fpga node cannot found\r\n");
      ret = -EINVAL;
      goto fail_findnd;
    }
    
    u64 regdata[2];
    /*2.获取reg属性内容*/
    ret = of_property_read_u64_array(fpga_acc.nd, "reg", regdata, 2);
    if(ret<0){
      printk("[fpga_acc drv] fail to red reg property\r\n");
      goto fail_rs;
    }

    fpga_acc.mmio_base = regdata[0];
    fpga_acc.mmio_size = regdata[1];
   
    printk("[fpga_acc drv] mmio base: 0x%llx\r\n",fpga_acc.mmio_base);
    printk("[fpga_acc drv] mmio size: 0x%llx\r\n",fpga_acc.mmio_size);

    return 0;

fail_rs:
fail_findnd:
    device_destroy(fpga_acc.class, fpga_acc.devno);
fail_device:
    class_destroy(fpga_acc.class);
fail_class:
    cdev_del(&fpga_acc.cdev);
fail_cdev:
    unregister_chrdev_region(fpga_acc.devno, 1);
fail_devno:
    return ret;
}

static void __exit fpga_acc_exit(void)
{
  //printk("[fpga_acc drv] driver remove\r\n");
  /* 删除字符设备 */
  cdev_del(&fpga_acc.cdev);

  /* 释放字符设号 */
	unregister_chrdev_region(fpga_acc.devno, 1); 

  /* 摧毁设备 */
  device_destroy(fpga_acc.class, fpga_acc.devno);
  
  /* 摧毁类 */
  class_destroy(fpga_acc.class);
}

module_init(fpga_acc_init);
module_exit(fpga_acc_exit);
