#ifndef COIN_SOLOCATION2EVENT_H
#define COIN_SOLOCATION2EVENT_H

/**************************************************************************\
 *
 *  This file is part of the Coin 3D visualization library.
 *  Copyright (C) 1998-2006 by Systems in Motion.  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  ("GPL") version 2 as published by the Free Software Foundation.
 *  See the file LICENSE.GPL at the root directory of this source
 *  distribution for additional information about the GNU GPL.
 *
 *  For using Coin with software that can not be combined with the GNU
 *  GPL, and for taking advantage of the additional benefits of our
 *  support services, please contact Systems in Motion about acquiring
 *  a Coin Professional Edition License.
 *
 *  See http://www.coin3d.org/ for more information.
 *
 *  Systems in Motion, Postboks 1283, Pirsenteret, 7462 Trondheim, NORWAY.
 *  http://www.sim.no/  sales@sim.no  coin-support@coin3d.org
 *
\**************************************************************************/

#include <Inventor/events/SoSubEvent.h>

class COIN_DLL_API SoLocation2Event : public SoEvent {
  typedef SoEvent inherited;

  SO_EVENT_HEADER();

public:
#ifdef COIN_NEXT_MINOR
  SoLocation2Event(void);
#endif // COIN_NEXT_MINOR
  virtual ~SoLocation2Event();

  static void initClass(void);
};

#endif // !COIN_SOLOCATION2EVENT_H
