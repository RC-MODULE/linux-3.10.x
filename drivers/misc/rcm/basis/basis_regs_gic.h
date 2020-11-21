#ifndef DEVICES_GIC_H
#define DEVICES_GIC_H

/**
 * \defgroup devices_irq_gic PL390 GIC
 * \ingroup devices_irq
 *
 * Register offsets and constants
 * \code{.c}
 * #include <regs/regs_gic.h>
 * \endcode
 *
 * \defgroup devices_irq_gic_regs Register offsets
 * \ingroup devices_irq_gic
 *
 * \addtogroup devices_irq_gic_regs
 * @{
 */
#define GICC_REG_IAR     0x00C
#define GICC_REG_EOIR    0x010

//#define GICD_REG_ISENABLER0 0x100
#define GICD_REG_ISENABLER1 0x100
#define GICD_REG_ISENABLER2 0x104
#define GICD_REG_ISENABLER3 0x108

//#define GICD_REG_ICENABLER0 0x180
#define GICD_REG_ICENABLER1 0x180
#define GICD_REG_ICENABLER2 0x184
#define GICD_REG_ICENABLER3 0x188

#define GICC_REG_CTLR 0x000
#define GICD_REG_CTLR 0x000
#define GICD_REG_SGIR 0xF00
#define GICD_REG_IGROUPR0 0x080
#define GICD_REG_IGROUPR1 0x084
#define GICD_REG_IGROUPR2 0x088
#define GICD_REG_TYPER 0x004
#define GICC_REG_PMR 0x004
#define GICD_REG_ICFGR0 0xC00
#define GICD_REG_ICFGR1 0xC04
#define GICD_REG_ICFGR2 0xC08
#define GICD_REG_ICFGR3 0xC0C
#define GICD_REG_ICFGR4 0xC10
#define GICD_REG_ICFGR5 0xC14

/**
 * @}
 * \defgroup devices_irq_gic_constants Constants
 * \ingroup devices_irq_gic
 *
 * \addtogroup devices_irq_gic_constants
 * @{
 */

#define GIC_GENSWINT0 0x00010000
#define GIC_GENSWINT1 0x00010001
#define GIC_GENSWINT2 0x00010002
#define GIC_GENSWINT3 0x00010003
#define GIC_GENSWINT4 0x00010004
#define GIC_GENSWINT5 0x00010005
#define GIC_GENSWINT6 0x00010006
#define GIC_GENSWINT7 0x00010007

/**
 * @}
 */

#endif /* end of include guard: DEVICES_GIC_H */
