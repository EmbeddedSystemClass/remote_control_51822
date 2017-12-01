/******************************************************************************
 *
 * COPYRIGHT:
 *   Copyright (c)  2005-2050   GnsTime  Inc.    All rights reserved.
 *
 *   This is unpublished proprietary source code of GnsTime Inc.
 *   The copyright notice above does not evidence any actual or intended
 *   publication of such source code.
 *
 * FILE NAME:
 *   CLI_CmdFile.h
 * DESCRIPTION:
 *   
 * HISTORY:
 *   2015年9月6日        Arvin         Create/Update
 *
*****************************************************************************/
#ifndef ECGSTP7_MIDDLEWARES_FILESSYS_CLI_CMDFILE_H_
#define ECGSTP7_MIDDLEWARES_FILESSYS_CLI_CMDFILE_H_


void
vRegisterFileSysCLICommands (void);

char *  cli_CmdGetPath(void);
#endif /* ECGSTP7_MIDDLEWARES_FILESSYS_CLI_CMDFILE_H_ */
