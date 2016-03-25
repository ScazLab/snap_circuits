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

NSVGgradient* nsvgCopyGradient(NSVGgradient* _g)
{
    NSVGgradient* res = (NSVGgradient*)malloc(sizeof(NSVGgradient));

    for (int i = 0; i < 6; ++i)
    {
        res->xform[i] = _g->xform[i];
    }
    res->spread   = _g->spread;
    res->fx       = _g->fx;
    res->fy       = _g->fy;
    res->nstops   = _g->nstops;

    res->stops[1].color  = _g->stops[1].color;
    res->stops[1].offset = _g->stops[1].offset;

    return res;
}

// NSVGshape* nsvgCopyShape(NSVGshape* _s)
// {
//     NSVGshape* res = (NSVGshape*)malloc(sizeof(NSVGshape));
//     strncpy(res->id, _s->id, 64);
    

//     char id[64];                // Optional 'id' attr of the shape or its group
//     NSVGpaint fill;             // Fill paint
//     NSVGpaint stroke;           // Stroke paint
//     float opacity;              // Opacity of the shape.
//     float strokeWidth;          // Stroke width (scaled).
//     float strokeDashOffset;     // Stroke dash offset (scaled).
//     float strokeDashArray[8];           // Stroke dash array (scaled).
//     char strokeDashCount;               // Number of dash values in dash array.
//     char strokeLineJoin;        // Stroke join type.
//     char strokeLineCap;         // Stroke cap type.
//     char fillRule;              // Fill rule, see NSVGfillRule.
//     unsigned char flags;        // Logical or of NSVG_FLAGS_* flags
//     float bounds[4];            // Tight bounding box of the shape [minx,miny,maxx,maxy].
//     NSVGpath* paths;            // Linked list of paths in the image.
//     struct NSVGshape* next; 
// }

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

void nsvgAppendShape(NSVGimage *_i, NSVGshape* _s)
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

void printNSVGpath(NSVGpath* _p)
{
    printf("Number of points: %i\nPoints: ", _p->npts);

    for (int i = 0; i < _p->npts; ++i)
    {
        printf("%f\t", _p->pts[i]);
    }
    printf("\nClosed: %i\t Bounds: %f %f %f %f\n",_p->closed,_p->bounds[1],_p->bounds[2],_p->bounds[3],_p->bounds[4]);
}

void printNSVGshape(NSVGshape* _s)
{
    printf("ID: %s\n", _s->id);

    for (NSVGpath* path = _s->paths; path != NULL; path = path->next)
    {
        printNSVGpath(path);
        printNSVGpath(nsvgDuplicatePath(path));
    }
}

#endif
