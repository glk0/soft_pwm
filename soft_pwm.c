#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

/*
 * soft-pwm.c: pwm signal generator with generic gpio ports
 * based on pwm-gpio
 * provides the following features over pwm-gpio:
 *  - compatibility with the latest available rpi kernel at the : this was
 *      was targeted to the Linux 6.6.47 who was the latest version of the kernel
 *      source maintained by the Raspberry foundation; at the time, pwm-gpio works
 *      only with pwmlib API specific to the Linux 6.10 kernel.
 *  - runtime acquisition of gpio pins used: instead of modifying the device
 *      tree and assigning  some pin to the PWM feature, we dynamically request and
 *      free gpio lines through gpiolib.
 *  - reduced resource use: all the channels are controlled by a single virtual
 *      controller, and the driver releases gpio lines and all associated
 *      resources when the corresponding pwm lines are unexported.
 */

#include <linux/module.h>
#include <linux/container_of.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/math.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>
#include <linux/pwm.h>

#define MAX_GPIO_PIN_IDX 27
#define BASE_GPIO_IDX 512

struct soft_pwm_chan {
    struct hrtimer timer;
    struct gpio_desc* gpio;
    struct pwm_state state;
    struct pwm_state next_state;

    spinlock_t lock;

    bool running;
    bool level;
    bool changing;
};


struct platform_device* pdev;
struct soft_pwm_chan* chans;

struct platform_driver soft_pwm_driver = {
    .driver = {
        .name = KBUILD_MODNAME,
    }
};

static u64 soft_pwm_toggle_gpio (struct soft_pwm_chan* chan, bool level) {
    const struct pwm_state* state = &chan->state;
    bool invert = state->polarity ==  PWM_POLARITY_INVERSED;

    chan->level = level;
    gpiod_set_value (chan->gpio, chan->level ^ invert);

    if (!state->duty_cycle || state->duty_cycle == state->period) {
        chan->running = false;
        return 0;
    }

    chan->running = true;
    return level ? state->duty_cycle : state->period - state->duty_cycle;
}

static enum hrtimer_restart soft_pwm_timer_func (struct hrtimer* timer)
{
    struct soft_pwm_chan* chan = container_of (timer, struct soft_pwm_chan, timer);
    u64 next_toggle;
    bool new_level;
    unsigned long flags;

    spin_lock_irqsave (&chan->lock, flags);

    if (!chan->level && chan->changing) {
        chan->changing  = false;
        chan->state = chan->next_state;
        new_level = !!chan->state.duty_cycle;
    } else {
        new_level = !chan->level;
    }

    next_toggle = soft_pwm_toggle_gpio (chan, new_level);
    if (next_toggle)
        hrtimer_forward (timer, hrtimer_get_expires(timer), ns_to_ktime(next_toggle));

    spin_unlock_irqrestore (&chan->lock, flags);
    return next_toggle ? HRTIMER_RESTART : HRTIMER_NORESTART;
}

static void soft_pwm_round (struct pwm_state *dest, const struct pwm_state *src)
{
    u64 dividend;
    u32 remainder;

    *dest = *src;

    dividend = dest->period;
    remainder = do_div(dividend, hrtimer_resolution);
    dest->period -= remainder;

    dividend = dest->duty_cycle;
    remainder = do_div(dividend, hrtimer_resolution);
    dest->duty_cycle -= remainder;

}

static int soft_pwm_apply (struct pwm_chip* chip, struct pwm_device* pwm,
        const struct pwm_state* state)
{
    unsigned long flags;
    struct soft_pwm_chan* chan = &chans [pwm->hwpwm];
    bool invert = state->polarity == PWM_POLARITY_INVERSED;


    if (state->duty_cycle && state->duty_cycle < hrtimer_resolution)
        return -EINVAL;

    if (state->duty_cycle != state->period &&
        (state->period - state->duty_cycle < hrtimer_resolution))
        return -EINVAL;

    if (!state->enabled)
        hrtimer_cancel (&chan->timer);
    else if (!chan->running)
        gpiod_set_value (chan->gpio, invert);

    spin_lock_irqsave (&chan->lock, flags);

    if (!state->enabled) {
        soft_pwm_round(&chan->state, state);
        chan->running = false;
        chan->changing = false;
    } else if (chan->running) {
        soft_pwm_round (&chan->next_state, state);
        chan->changing = true;
    } else {
        unsigned long next_toggle;
        soft_pwm_round (&chan->state, state);
        chan->changing = false;

        next_toggle = soft_pwm_toggle_gpio(chan, !!state->duty_cycle);
        if (next_toggle)
            hrtimer_start (&chan->timer, next_toggle, HRTIMER_MODE_REL);
    }
    spin_unlock_irqrestore (&chan->lock, flags);
    return 0;
}

static int soft_pwm_get_state (struct pwm_chip* chip,
        struct pwm_device* pwm, struct pwm_state* state)
{
    unsigned long flags;
    struct soft_pwm_chan* chan = &chans [pwm->hwpwm];

    spin_lock_irqsave(&chan->lock, flags);
    if (chan->changing)
        *state = chan->state;
    else
        *state = chan->next_state;
    spin_unlock_irqrestore(&chan->lock, flags);
    return 0;
}

static int soft_pwm_request (struct pwm_chip* chip, struct pwm_device* pwm)
{
    unsigned long flags;
    int err = 0;
    unsigned gpio_num = BASE_GPIO_IDX + pwm->hwpwm;
    struct gpio_desc* gdesc;
    struct soft_pwm_chan* chan;

    if (pwm->hwpwm > MAX_GPIO_PIN_IDX || pwm->hwpwm < 0)
        return -EINVAL;

    chan = &chans [pwm->hwpwm];

    spin_lock_irqsave (&chan->lock, flags);

    err = gpio_request_one (gpio_num, GPIOF_OUT_INIT_LOW, NULL);
    if (err)
        goto release_lock;

    gdesc = gpio_to_desc (gpio_num);
    if ((gdesc == NULL) || gpiod_cansleep (gdesc)) {
        err = -EINVAL;
        goto gpio_error;
    }

    chan->gpio = gdesc;

    hrtimer_init (&chan->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    chan->timer.function = soft_pwm_timer_func;

    goto release_lock;

gpio_error:
    gpio_free (gpio_num);
release_lock:
    spin_unlock_irqrestore (&chan->lock, flags);
    return err;
}

static void soft_pwm_free (struct pwm_chip* chip, struct pwm_device* pwm)
{
    unsigned long flags;
    unsigned gpio_num = BASE_GPIO_IDX + pwm->hwpwm;
    struct soft_pwm_chan* chan = &chans [pwm->hwpwm];

    spin_lock_irqsave (&chan->lock, flags);

    gpio_free (gpio_num);
    hrtimer_cancel (&chan->timer);

    spin_unlock_irqrestore (&chan->lock, flags);
}


static const struct pwm_ops soft_pwm_ops  = {
    .apply = soft_pwm_apply,
    .get_state = soft_pwm_get_state,
    .free = soft_pwm_free,
    .request = soft_pwm_request,
};

static int __init soft_pwm_probe (struct platform_device *pdev)
{
    int ret;
    struct pwm_chip* chip;

    chip = devm_kzalloc (&pdev->dev, sizeof(*chip), GFP_KERNEL);
    if (!chip)
        return -ENOMEM;

    chans = devm_kcalloc (&pdev->dev, MAX_GPIO_PIN_IDX + 1, sizeof (*chans), GFP_KERNEL);
    if (!chans)
        return -ENOMEM;

    chip->dev = &pdev->dev;
    chip->ops = &soft_pwm_ops;
    chip->atomic = true;
    chip->npwm = MAX_GPIO_PIN_IDX + 1;

    ret = devm_pwmchip_add (&pdev->dev, chip);
    if (ret < 0)
        return dev_err_probe (&pdev->dev, ret, "failed to add pwmchip\n");

    for (int idx = 0; idx <= MAX_GPIO_PIN_IDX; ++ idx)
        spin_lock_init (&chans [idx].lock);

    return 0;
}

static int __init soft_pwm_init (void)
{
    int ret;
    pdev = platform_device_register_simple (KBUILD_MODNAME, -1, NULL, 0);
    if (IS_ERR (pdev)) {
        pr_err ("platform device allocation failed\n");
        return PTR_ERR (pdev);
    }
    ret = platform_driver_probe(&soft_pwm_driver, soft_pwm_probe);
    if (ret) {
        pr_err ("platform driver probing failed");
        platform_device_unregister (pdev);
    }
    return ret;
}

static void __exit soft_pwm_exit (void)
{
    platform_device_unregister (pdev);
    platform_driver_unregister (&soft_pwm_driver);
}

module_init(soft_pwm_init);
module_exit(soft_pwm_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kossi GLOKPOR");
MODULE_DESCRIPTION("Driver for PWM on the 27 Raspberry PI 4-b GPIO ports");
