/*
 * Copyright (c) 2015 Thomas Roell  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Thomas Roell, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#if !defined(SIMULATION)
#include "kitty.h"
#endif /* SIMULATION */

const waypoint_t waypoints[] = {
  { 400906357,-1051857542,1, },
{ 400906706,-1051857951,1, },
{ 400907075,-1051858051,1, },
{ 400907440,-1051857843,2, },
{ 400909989,-1051854920,2, },
{ 400910292,-1051854450,1, },
{ 400910661,-1051853981,1, },
{ 400911205,-1051853351,1, },
{ 400911800,-1051852680,1, },
{ 400912120,-1051852441,1, },
{ 400912467,-1051852197,1, },
{ 400912759,-1051851922,1, },
{ 400912821,-1051851520,1, },
{ 400912534,-1051850903,1, },
{ 400912205,-1051850548,1, },
{ 400911903,-1051850058,1, },
{ 400911626,-1051849602,1, },
{ 400911015,-1051848650,1, },
{ 400910666,-1051848583,1, },
{ 400910307,-1051848824,2, },
{ 400909661,-1051849870,2, },
{ 400908973,-1051850608,2, },
{ 400907650,-1051851963,3, },
{ 400906101,-1051853465,1, },
{ 400905331,-1051854296,1, },
{ 400904982,-1051854712,1, },
{ 400904911,-1051855087,5, },
{ 400905044,-1051855570,1, },
{ 400905300,-1051855905,1, },
{ 400906794,-1051858107,6, },
};

#if 0
const waypoint_t waypoints[] = {
    { 400906357,-1051857542,1, },
    { 400906706,-1051857951,1, },
    { 400907075,-1051858051,1, },
    { 400907440,-1051857843,2, },
    { 400909989,-1051854920,2, },
    { 400910292,-1051854450,1, },
    { 400910646,-1051853947,1, },
    { 400911215,-1051853290,1, },
    { 400911790,-1051852673,1, },
    { 400912120,-1051852441,1, },
    { 400912467,-1051852197,1, },
    { 400912759,-1051851922,1, },
    { 400912821,-1051851520,1, },
    { 400912534,-1051850903,1, },
    { 400912226,-1051850541,1, },
    { 400911903,-1051850025,1, },
    { 400911620,-1051849562,1, },
    { 400911015,-1051848650,1, },
    { 400910666,-1051848583,1, },
    { 400910307,-1051848824,2, },
    { 400907797,-1051851645,3, },
    { 400906101,-1051853465,1, },
    { 400905331,-1051854296,1, },
    { 400904982,-1051854712,1, },
    { 400904911,-1051855087,5, },
    { 400905044,-1051855570,1, },
    { 400905300,-1051855905,1, },
    { 400906794,-1051858107,6, },
};
#endif

#if 1

const mission_t mission = {
    400909271,            /* latitude  */
    -1051850339,          /* longitude */
    1610,                 /* altitude  */
    8.61 * DEG2RAD,       /* variation */
    sizeof(waypoints) / sizeof(waypoints[0]),
    &waypoints[0],
    {
	0.0,              /* 0 STOP   */
	4.0,              /* 1 SLOW   */
	6.0,              /* 2 MEDIUM */
	8.0,              /* 3 FAST   */
	10.0,             /* 4 FULL   */
	1.0,              /* 5 BRAKE  */
	2.8,              /* 6 COAST  */
    },
};

#endif

#if 0

const mission_t mission = {
    396880731,            /* latitude  */
    -1049681311,          /* longitude */
    1610,                 /* altitude  */
    8.51708 * DEG2RAD,    /* variation */
    sizeof(waypoints) / sizeof(waypoints[0]),
    &waypoints[0],
    {
	0.0,              /* STOP   */
	4.0,              /* SLOW   */
	6.0,              /* MEDIUM */
	8.0,              /* FAST   */
	10.0,             /* ULTRA  */
    },
};

#endif
