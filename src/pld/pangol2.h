/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2023 by Max A. Dednev                                   *
 *   mdednev@yandex.ru                                                     *
 ***************************************************************************/

#ifndef OPENOCD_PLD_PANGOL2_H
#define OPENOCD_PLD_PANGOL2_H

#include <jtag/jtag.h>

struct pangol2_pld_device {
	struct jtag_tap *tap;
	int no_jwakeup;
};

#endif /* OPENOCD_PLD_PANGOL2_H */
