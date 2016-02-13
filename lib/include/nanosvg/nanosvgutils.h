/**
 * Copyright (C) 2016 Social Robotics Lab, Yale University
 * Author: Alessandro Roncone
 * email:  alessandro.roncone@yale.edu
 * website: www.scazlab.yale.edu
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 3 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
**/

#ifndef NANOSVGUTILS_H
#define NANOSVGUTILS_H

#include "nanosvg/nanosvg.h"
#include <stdio.h>

void printNSVGpath(NSVGpath _p)
{
	printf("Number of points: %i\t", _p.npts);

	for (int i = 0; i < _p.npts; ++i)
	{
		printf("%f\t", _p.pts[i]);
	}
	printf("\n");
}

void printNSVGshape(NSVGshape _s)
{
	printf("ID: %s\n", _s.id);
}

NSVGshape* findLast(NSVGshape *_s)
{
    if (_s==NULL) return _s;

    NSVGshape* res=_s;

    while(res->next!=NULL)
    {
        res=res->next;
    }

    return res;
}

void appendNSVGshape(NSVGimage *_i, NSVGshape* _s)
{
    NSVGshape* last=findLast(_i->shapes);

    if (last==NULL)
    {
        _i->shapes=_s;
    }
    else
    {
        last->next=_s;
    }
}

#endif
