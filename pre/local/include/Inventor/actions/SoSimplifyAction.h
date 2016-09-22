#ifndef COIN_SOSIMPLIFYACTION_H
#define COIN_SOSIMPLIFYACTION_H

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

#include <Inventor/actions/SoAction.h>
#include <Inventor/actions/SoSubAction.h>

#ifdef COIN_NEXT_MINOR

class SoSimplifyActionP;

class COIN_DLL_API SoSimplifyAction : public SoAction {
  typedef SoAction inherited;

  SO_ACTION_HEADER(SoSimplifyAction);

public:
  SoSimplifyAction(void);
  virtual ~SoSimplifyAction();

  virtual void apply(SoNode * root);
  virtual void apply(SoPath * path);
  virtual void apply(const SoPathList & pathlist, SbBool obeysrules = FALSE);

  static void initClass(void);

protected:
  virtual void beginTraversal(SoNode * node);

private:
  SoSimplifyActionP * pimpl;
};

#endif // COIN_NEXT_MINOR

#endif // !COIN_SOSIMPLIFYACTION_H
