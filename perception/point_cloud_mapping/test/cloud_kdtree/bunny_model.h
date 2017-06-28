/*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: bunny_model.h 21050 2009-08-07 21:24:30Z jfaustwg $
 *
 */

/** \author Radu Bogdan Rusu */

#ifndef _CLOUD_KDTREE_BUNNYMODEL_H
#define _CLOUD_KDTREE_BUNNYMODEL_H_

#include "sensor_msgs/PointCloud.h"

namespace cloud_kdtree_tests
{

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Fill in a given point cloud message data structure with a downsampled copy of the Stanford bunny model
    * \param points the cloud data
    */
  void
    getBunnyModel (sensor_msgs::PointCloud &points)
  {
    points.points.resize (397);

    points.points[    0].x = 0.005422;  points.points[    0].y = 0.113490; points.points[    0].z = 0.040749;
    points.points[    1].x = -0.00174;  points.points[    1].y = 0.114250; points.points[    1].z = 0.041273;
    points.points[    2].x = -0.01066;  points.points[    2].y = 0.113380; points.points[    2].z = 0.040916;
    points.points[    3].x = 0.026422;  points.points[    3].y = 0.114990; points.points[    3].z = 0.032623;
    points.points[    4].x = 0.024545;  points.points[    4].y = 0.122840; points.points[    4].z = 0.024255;
    points.points[    5].x = 0.034137;  points.points[    5].y = 0.113160; points.points[    5].z = 0.025070;
    points.points[    6].x = 0.028860;  points.points[    6].y = 0.117730; points.points[    6].z = 0.027037;
    points.points[    7].x = 0.026750;  points.points[    7].y = 0.122340; points.points[    7].z = 0.017605;
    points.points[    8].x = 0.035750;  points.points[    8].y = 0.112300; points.points[    8].z = 0.019109;
    points.points[    9].x = 0.015982;  points.points[    9].y = 0.123070; points.points[    9].z = 0.031279;
    points.points[   10].x = 0.007981;  points.points[   10].y = 0.124380; points.points[   10].z = 0.032798;
    points.points[   11].x = 0.018101;  points.points[   11].y = 0.116740; points.points[   11].z = 0.035493;
    points.points[   12].x = 0.008669;  points.points[   12].y = 0.117580; points.points[   12].z = 0.037538;
    points.points[   13].x = 0.018080;  points.points[   13].y = 0.125360; points.points[   13].z = 0.026132;
    points.points[   14].x = 0.008086;  points.points[   14].y = 0.128660; points.points[   14].z = 0.026190;
    points.points[   15].x = 0.022750;  points.points[   15].y = 0.121460; points.points[   15].z = 0.029671;
    points.points[   16].x = -0.001869; points.points[   16].y = 0.124560; points.points[   16].z = 0.033184;
    points.points[   17].x = -0.011168; points.points[   17].y = 0.123760; points.points[   17].z = 0.032519;
    points.points[   18].x = -0.002006; points.points[   18].y = 0.119370; points.points[   18].z = 0.038104;
    points.points[   19].x = -0.012320; points.points[   19].y = 0.118160; points.points[   19].z = 0.037427;
    points.points[   20].x = -0.001666; points.points[   20].y = 0.128790; points.points[   20].z = 0.026782;
    points.points[   21].x = -0.011971; points.points[   21].y = 0.127230; points.points[   21].z = 0.026219;
    points.points[   22].x = 0.016484;  points.points[   22].y = 0.128280; points.points[   22].z = 0.019280;
    points.points[   23].x = 0.007092;  points.points[   23].y = 0.131030; points.points[   23].z = 0.018415;
    points.points[   24].x = 0.001461;  points.points[   24].y = 0.131340; points.points[   24].z = 0.017095;
    points.points[   25].x = -0.013821; points.points[   25].y = 0.128860; points.points[   25].z = 0.019265;
    points.points[   26].x = -0.017250; points.points[   26].y = 0.112020; points.points[   26].z = 0.040077;
    points.points[   27].x = -0.074556; points.points[   27].y = 0.134150; points.points[   27].z = 0.051046;
    points.points[   28].x = -0.065971; points.points[   28].y = 0.143960; points.points[   28].z = 0.041090;
    points.points[   29].x = -0.071925; points.points[   29].y = 0.145450; points.points[   29].z = 0.043266;
    points.points[   30].x = -0.065510; points.points[   30].y = 0.136240; points.points[   30].z = 0.042195;
    points.points[   31].x = -0.071112; points.points[   31].y = 0.137670; points.points[   31].z = 0.047518;
    points.points[   32].x = -0.079528; points.points[   32].y = 0.134160; points.points[   32].z = 0.051194;
    points.points[   33].x = -0.080421; points.points[   33].y = 0.144280; points.points[   33].z = 0.042793;
    points.points[   34].x = -0.082672; points.points[   34].y = 0.137800; points.points[   34].z = 0.046806;
    points.points[   35].x = -0.088130; points.points[   35].y = 0.135140; points.points[   35].z = 0.042222;
    points.points[   36].x = -0.066325; points.points[   36].y = 0.123470; points.points[   36].z = 0.050729;
    points.points[   37].x = -0.072399; points.points[   37].y = 0.126620; points.points[   37].z = 0.052364;
    points.points[   38].x = -0.066091; points.points[   38].y = 0.119730; points.points[   38].z = 0.050881;
    points.points[   39].x = -0.072012; points.points[   39].y = 0.118110; points.points[   39].z = 0.052295;
    points.points[   40].x = -0.062433; points.points[   40].y = 0.126270; points.points[   40].z = 0.043831;
    points.points[   41].x = -0.068326; points.points[   41].y = 0.129980; points.points[   41].z = 0.048875;
    points.points[   42].x = -0.063094; points.points[   42].y = 0.118110; points.points[   42].z = 0.044399;
    points.points[   43].x = -0.071301; points.points[   43].y = 0.113220; points.points[   43].z = 0.048410;
    points.points[   44].x = -0.080515; points.points[   44].y = 0.127410; points.points[   44].z = 0.052034;
    points.points[   45].x = -0.078179; points.points[   45].y = 0.119100; points.points[   45].z = 0.051116;
    points.points[   46].x = -0.085216; points.points[   46].y = 0.126090; points.points[   46].z = 0.049001;
    points.points[   47].x = -0.089538; points.points[   47].y = 0.126210; points.points[   47].z = 0.044589;
    points.points[   48].x = -0.082659; points.points[   48].y = 0.116610; points.points[   48].z = 0.047970;
    points.points[   49].x = -0.089536; points.points[   49].y = 0.117840; points.points[   49].z = 0.044570;
    points.points[   50].x = -0.056500; points.points[   50].y = 0.152480; points.points[   50].z = 0.030132;
    points.points[   51].x = -0.055517; points.points[   51].y = 0.153130; points.points[   51].z = 0.026915;
    points.points[   52].x = -0.036250; points.points[   52].y = 0.171980; points.points[   52].z = 0.000177;
    points.points[   53].x = -0.037750; points.points[   53].y = 0.171980; points.points[   53].z = 0.000222;
    points.points[   54].x = -0.036250; points.points[   54].y = 0.169350; points.points[   54].z = 0.000520;
    points.points[   55].x = -0.033176; points.points[   55].y = 0.157110; points.points[   55].z = 0.001868;
    points.points[   56].x = -0.051913; points.points[   56].y = 0.154500; points.points[   56].z = 0.011273;
    points.points[   57].x = -0.041707; points.points[   57].y = 0.166420; points.points[   57].z = 0.003052;
    points.points[   58].x = -0.049468; points.points[   58].y = 0.164140; points.points[   58].z = 0.004199;
    points.points[   59].x = -0.041892; points.points[   59].y = 0.156690; points.points[   59].z = 0.005488;
    points.points[   60].x = -0.051224; points.points[   60].y = 0.158780; points.points[   60].z = 0.008028;
    points.points[   61].x = -0.062417; points.points[   61].y = 0.153170; points.points[   61].z = 0.033161;
    points.points[   62].x = -0.071670; points.points[   62].y = 0.153190; points.points[   62].z = 0.033701;
    points.points[   63].x = -0.062543; points.points[   63].y = 0.155240; points.points[   63].z = 0.027405;
    points.points[   64].x = -0.072110; points.points[   64].y = 0.155500; points.points[   64].z = 0.027645;
    points.points[   65].x = -0.078663; points.points[   65].y = 0.152690; points.points[   65].z = 0.032268;
    points.points[   66].x = -0.081569; points.points[   66].y = 0.153740; points.points[   66].z = 0.026085;
    points.points[   67].x = -0.087250; points.points[   67].y = 0.152300; points.points[   67].z = 0.022135;
    points.points[   68].x = -0.057250; points.points[   68].y = 0.155680; points.points[   68].z = 0.010325;
    points.points[   69].x = -0.057888; points.points[   69].y = 0.157500; points.points[   69].z = 0.007322;
    points.points[   70].x = -0.088500; points.points[   70].y = 0.152230; points.points[   70].z = 0.019215;
    points.points[   71].x = -0.056129; points.points[   71].y = 0.146160; points.points[   71].z = 0.030850;
    points.points[   72].x = -0.054705; points.points[   72].y = 0.135550; points.points[   72].z = 0.032127;
    points.points[   73].x = -0.054144; points.points[   73].y = 0.147140; points.points[   73].z = 0.026275;
    points.points[   74].x = -0.046625; points.points[   74].y = 0.132340; points.points[   74].z = 0.021909;
    points.points[   75].x = -0.051390; points.points[   75].y = 0.136940; points.points[   75].z = 0.025787;
    points.points[   76].x = -0.018278; points.points[   76].y = 0.122380; points.points[   76].z = 0.030773;
    points.points[   77].x = -0.021656; points.points[   77].y = 0.116430; points.points[   77].z = 0.035209;
    points.points[   78].x = -0.031921; points.points[   78].y = 0.115660; points.points[   78].z = 0.032851;
    points.points[   79].x = -0.021348; points.points[   79].y = 0.124210; points.points[   79].z = 0.024562;
    points.points[   80].x = -0.032410; points.points[   80].y = 0.123490; points.points[   80].z = 0.023293;
    points.points[   81].x = -0.024869; points.points[   81].y = 0.120940; points.points[   81].z = 0.028745;
    points.points[   82].x = -0.031747; points.points[   82].y = 0.120390; points.points[   82].z = 0.028229;
    points.points[   83].x = -0.052912; points.points[   83].y = 0.126860; points.points[   83].z = 0.034968;
    points.points[   84].x = -0.041672; points.points[   84].y = 0.115640; points.points[   84].z = 0.032998;
    points.points[   85].x = -0.052037; points.points[   85].y = 0.116800; points.points[   85].z = 0.034582;
    points.points[   86].x = -0.042495; points.points[   86].y = 0.124880; points.points[   86].z = 0.024082;
    points.points[   87].x = -0.047946; points.points[   87].y = 0.127360; points.points[   87].z = 0.028108;
    points.points[   88].x = -0.042421; points.points[   88].y = 0.120350; points.points[   88].z = 0.028633;
    points.points[   89].x = -0.047661; points.points[   89].y = 0.120240; points.points[   89].z = 0.028871;
    points.points[   90].x = -0.035964; points.points[   90].y = 0.151300; points.points[   90].z = 0.000539;
    points.points[   91].x = -0.050598; points.points[   91].y = 0.147400; points.points[   91].z = 0.013881;
    points.points[   92].x = -0.046375; points.points[   92].y = 0.132930; points.points[   92].z = 0.018289;
    points.points[   93].x = -0.049125; points.points[   93].y = 0.138560; points.points[   93].z = 0.016269;
    points.points[   94].x = -0.042976; points.points[   94].y = 0.149150; points.points[   94].z = 0.005400;
    points.points[   95].x = -0.047965; points.points[   95].y = 0.146590; points.points[   95].z = 0.008678;
    points.points[   96].x = -0.022926; points.points[   96].y = 0.126300; points.points[   96].z = 0.018077;
    points.points[   97].x = -0.031583; points.points[   97].y = 0.125900; points.points[   97].z = 0.017804;
    points.points[   98].x = -0.041733; points.points[   98].y = 0.127960; points.points[   98].z = 0.016650;
    points.points[   99].x = -0.061482; points.points[   99].y = 0.146980; points.points[   99].z = 0.036168;
    points.points[  100].x = -0.071729; points.points[  100].y = 0.150260; points.points[  100].z = 0.038328;
    points.points[  101].x = -0.060526; points.points[  101].y = 0.136800; points.points[  101].z = 0.035999;
    points.points[  102].x = -0.082619; points.points[  102].y = 0.148230; points.points[  102].z = 0.035955;
    points.points[  103].x = -0.087824; points.points[  103].y = 0.144490; points.points[  103].z = 0.033779;
    points.points[  104].x = -0.089000; points.points[  104].y = 0.138280; points.points[  104].z = 0.037774;
    points.points[  105].x = -0.085662; points.points[  105].y = 0.150950; points.points[  105].z = 0.028208;
    points.points[  106].x = -0.089601; points.points[  106].y = 0.147250; points.points[  106].z = 0.025869;
    points.points[  107].x = -0.090681; points.points[  107].y = 0.137480; points.points[  107].z = 0.023690;
    points.points[  108].x = -0.058722; points.points[  108].y = 0.129240; points.points[  108].z = 0.038992;
    points.points[  109].x = -0.060075; points.points[  109].y = 0.115120; points.points[  109].z = 0.037685;
    points.points[  110].x = -0.091812; points.points[  110].y = 0.127670; points.points[  110].z = 0.038703;
    points.points[  111].x = -0.091727; points.points[  111].y = 0.116570; points.points[  111].z = 0.039619;
    points.points[  112].x = -0.093164; points.points[  112].y = 0.127210; points.points[  112].z = 0.025211;
    points.points[  113].x = -0.093938; points.points[  113].y = 0.120670; points.points[  113].z = 0.024399;
    points.points[  114].x = -0.091583; points.points[  114].y = 0.145220; points.points[  114].z = 0.019860;
    points.points[  115].x = -0.090929; points.points[  115].y = 0.136670; points.points[  115].z = 0.019817;
    points.points[  116].x = -0.093094; points.points[  116].y = 0.116350; points.points[  116].z = 0.018959;
    points.points[  117].x = 0.024948;  points.points[  117].y = 0.102860; points.points[  117].z = 0.041418;
    points.points[  118].x = 0.033600;  points.points[  118].y = 0.092627; points.points[  118].z = 0.040463;
    points.points[  119].x = 0.027420;  points.points[  119].y = 0.096386; points.points[  119].z = 0.043312;
    points.points[  120].x = 0.033920;  points.points[  120].y = 0.086911; points.points[  120].z = 0.041034;
    points.points[  121].x = 0.028156;  points.points[  121].y = 0.086837; points.points[  121].z = 0.045084;
    points.points[  122].x = 0.033810;  points.points[  122].y = 0.078604; points.points[  122].z = 0.040854;
    points.points[  123].x = 0.028125;  points.points[  123].y = 0.076874; points.points[  123].z = 0.045059;
    points.points[  124].x = 0.014500;  points.points[  124].y = 0.093279; points.points[  124].z = 0.050880;
    points.points[  125].x = 0.007482;  points.points[  125].y = 0.094730; points.points[  125].z = 0.052315;
    points.points[  126].x = 0.017407;  points.points[  126].y = 0.105350; points.points[  126].z = 0.043139;
    points.points[  127].x = 0.007954;  points.points[  127].y = 0.106330; points.points[  127].z = 0.042968;
    points.points[  128].x = 0.018511;  points.points[  128].y = 0.097194; points.points[  128].z = 0.047253;
    points.points[  129].x = 0.008644;  points.points[  129].y = 0.099323; points.points[  129].z = 0.048079;
    points.points[  130].x = -0.002020; points.points[  130].y = 0.095698; points.points[  130].z = 0.053906;
    points.points[  131].x = -0.011446; points.points[  131].y = 0.095169; points.points[  131].z = 0.053862;
    points.points[  132].x = -0.001875; points.points[  132].y = 0.106910; points.points[  132].z = 0.043455;
    points.points[  133].x = -0.011875; points.points[  133].y = 0.106880; points.points[  133].z = 0.043019;
    points.points[  134].x = -0.001762; points.points[  134].y = 0.100710; points.points[  134].z = 0.046648;
    points.points[  135].x = -0.012498; points.points[  135].y = 0.100080; points.points[  135].z = 0.045916;
    points.points[  136].x = 0.016381;  points.points[  136].y = 0.085894; points.points[  136].z = 0.051642;
    points.points[  137].x = 0.008117;  points.points[  137].y = 0.086910; points.points[  137].z = 0.055228;
    points.points[  138].x = 0.017644;  points.points[  138].y = 0.076955; points.points[  138].z = 0.052372;
    points.points[  139].x = 0.008125;  points.points[  139].y = 0.076853; points.points[  139].z = 0.055536;
    points.points[  140].x = 0.020575;  points.points[  140].y = 0.088169; points.points[  140].z = 0.049006;
    points.points[  141].x = 0.022445;  points.points[  141].y = 0.075721; points.points[  141].z = 0.049563;
    points.points[  142].x = -0.001793; points.points[  142].y = 0.086849; points.points[  142].z = 0.056843;
    points.points[  143].x = -0.011943; points.points[  143].y = 0.086771; points.points[  143].z = 0.057009;
    points.points[  144].x = -0.001957; points.points[  144].y = 0.076863; points.points[  144].z = 0.057803;
    points.points[  145].x = -0.011875; points.points[  145].y = 0.076964; points.points[  145].z = 0.057022;
    points.points[  146].x = 0.033250;  points.points[  146].y = 0.067541; points.points[  146].z = 0.040033;
    points.points[  147].x = 0.028149;  points.points[  147].y = 0.066829; points.points[  147].z = 0.042953;
    points.points[  148].x = 0.026761;  points.points[  148].y = 0.057829; points.points[  148].z = 0.042588;
    points.points[  149].x = 0.023571;  points.points[  149].y = 0.047460; points.points[  149].z = 0.040428;
    points.points[  150].x = 0.015832;  points.points[  150].y = 0.067418; points.points[  150].z = 0.051639;
    points.points[  151].x = 0.008043;  points.points[  151].y = 0.066902; points.points[  151].z = 0.055006;
    points.points[  152].x = 0.013984;  points.points[  152].y = 0.058886; points.points[  152].z = 0.050416;
    points.points[  153].x = 0.008097;  points.points[  153].y = 0.056888; points.points[  153].z = 0.052950;
    points.points[  154].x = 0.020566;  points.points[  154].y = 0.065958; points.points[  154].z = 0.048300;
    points.points[  155].x = 0.018594;  points.points[  155].y = 0.056539; points.points[  155].z = 0.047879;
    points.points[  156].x = 0.012875;  points.points[  156].y = 0.052652; points.points[  156].z = 0.049689;
    points.points[  157].x = -0.001785; points.points[  157].y = 0.066712; points.points[  157].z = 0.056503;
    points.points[  158].x = -0.011785; points.points[  158].y = 0.066885; points.points[  158].z = 0.055015;
    points.points[  159].x = -0.001875; points.points[  159].y = 0.056597; points.points[  159].z = 0.054410;
    points.points[  160].x = -0.011840; points.points[  160].y = 0.057054; points.points[  160].z = 0.052714;
    points.points[  161].x = -0.015688; points.points[  161].y = 0.052469; points.points[  161].z = 0.049615;
    points.points[  162].x = 0.006615;  points.points[  162].y = 0.049930; points.points[  162].z = 0.051259;
    points.points[  163].x = 0.018088;  points.points[  163].y = 0.046655; points.points[  163].z = 0.043321;
    points.points[  164].x = 0.008841;  points.points[  164].y = 0.045437; points.points[  164].z = 0.046623;
    points.points[  165].x = 0.017688;  points.points[  165].y = 0.039719; points.points[  165].z = 0.043084;
    points.points[  166].x = 0.008125;  points.points[  166].y = 0.039516; points.points[  166].z = 0.045374;
    points.points[  167].x = -0.001611; points.points[  167].y = 0.049844; points.points[  167].z = 0.051720;
    points.points[  168].x = -0.012450; points.points[  168].y = 0.046773; points.points[  168].z = 0.050903;
    points.points[  169].x = -0.013851; points.points[  169].y = 0.039778; points.points[  169].z = 0.051036;
    points.points[  170].x = -0.002029; points.points[  170].y = 0.044874; points.points[  170].z = 0.047587;
    points.points[  171].x = -0.011653; points.points[  171].y = 0.046860; points.points[  171].z = 0.048661;
    points.points[  172].x = -0.001861; points.points[  172].y = 0.039606; points.points[  172].z = 0.047339;
    points.points[  173].x = -0.009155; points.points[  173].y = 0.039580; points.points[  173].z = 0.049415;
    points.points[  174].x = 0.043661;  points.points[  174].y = 0.094028; points.points[  174].z = 0.022520;
    points.points[  175].x = 0.034642;  points.points[  175].y = 0.104730; points.points[  175].z = 0.031831;
    points.points[  176].x = 0.028343;  points.points[  176].y = 0.107200; points.points[  176].z = 0.036339;
    points.points[  177].x = 0.036339;  points.points[  177].y = 0.096552; points.points[  177].z = 0.034843;
    points.points[  178].x = 0.031733;  points.points[  178].y = 0.099372; points.points[  178].z = 0.038505;
    points.points[  179].x = 0.036998;  points.points[  179].y = 0.106680; points.points[  179].z = 0.026781;
    points.points[  180].x = 0.032875;  points.points[  180].y = 0.111080; points.points[  180].z = 0.029590;
    points.points[  181].x = 0.040938;  points.points[  181].y = 0.097132; points.points[  181].z = 0.026663;
    points.points[  182].x = 0.044153;  points.points[  182].y = 0.086466; points.points[  182].z = 0.024241;
    points.points[  183].x = 0.053750;  points.points[  183].y = 0.072221; points.points[  183].z = 0.020429;
    points.points[  184].x = 0.045160;  points.points[  184].y = 0.076574; points.points[  184].z = 0.023594;
    points.points[  185].x = 0.038036;  points.points[  185].y = 0.086663; points.points[  185].z = 0.035459;
    points.points[  186].x = 0.037861;  points.points[  186].y = 0.076625; points.points[  186].z = 0.035658;
    points.points[  187].x = 0.042216;  points.points[  187].y = 0.087237; points.points[  187].z = 0.028254;
    points.points[  188].x = 0.042355;  points.points[  188].y = 0.076747; points.points[  188].z = 0.028580;
    points.points[  189].x = 0.043875;  points.points[  189].y = 0.096228; points.points[  189].z = 0.015269;
    points.points[  190].x = 0.044375;  points.points[  190].y = 0.096797; points.points[  190].z = 0.008644;
    points.points[  191].x = 0.039545;  points.points[  191].y = 0.106100; points.points[  191].z = 0.017655;
    points.points[  192].x = 0.042313;  points.points[  192].y = 0.100090; points.points[  192].z = 0.017237;
    points.points[  193].x = 0.045406;  points.points[  193].y = 0.087417; points.points[  193].z = 0.015604;
    points.points[  194].x = 0.055118;  points.points[  194].y = 0.072639; points.points[  194].z = 0.017944;
    points.points[  195].x = 0.048722;  points.points[  195].y = 0.073760; points.points[  195].z = 0.017434;
    points.points[  196].x = 0.045917;  points.points[  196].y = 0.086298; points.points[  196].z = 0.009421;
    points.points[  197].x = 0.019433;  points.points[  197].y = 0.109600; points.points[  197].z = 0.039063;
    points.points[  198].x = 0.010970;  points.points[  198].y = 0.110580; points.points[  198].z = 0.039648;
    points.points[  199].x = 0.046657;  points.points[  199].y = 0.057153; points.points[  199].z = 0.031337;
    points.points[  200].x = 0.056079;  points.points[  200].y = 0.066335; points.points[  200].z = 0.024122;
    points.points[  201].x = 0.048168;  points.points[  201].y = 0.067010; points.points[  201].z = 0.026298;
    points.points[  202].x = 0.056055;  points.points[  202].y = 0.057253; points.points[  202].z = 0.024902;
    points.points[  203].x = 0.051163;  points.points[  203].y = 0.056662; points.points[  203].z = 0.029137;
    points.points[  204].x = 0.036914;  points.points[  204].y = 0.067032; points.points[  204].z = 0.036122;
    points.points[  205].x = 0.033000;  points.points[  205].y = 0.064720; points.points[  205].z = 0.039903;
    points.points[  206].x = 0.038004;  points.points[  206].y = 0.056507; points.points[  206].z = 0.033119;
    points.points[  207].x = 0.030629;  points.points[  207].y = 0.054915; points.points[  207].z = 0.038484;
    points.points[  208].x = 0.041875;  points.points[  208].y = 0.066383; points.points[  208].z = 0.028357;
    points.points[  209].x = 0.041434;  points.points[  209].y = 0.060880; points.points[  209].z = 0.029632;
    points.points[  210].x = 0.044921;  points.points[  210].y = 0.049904; points.points[  210].z = 0.031243;
    points.points[  211].x = 0.054635;  points.points[  211].y = 0.050167; points.points[  211].z = 0.022044;
    points.points[  212].x = 0.048280;  points.points[  212].y = 0.047370; points.points[  212].z = 0.025845;
    points.points[  213].x = 0.037973;  points.points[  213].y = 0.048347; points.points[  213].z = 0.031456;
    points.points[  214].x = 0.028053;  points.points[  214].y = 0.047061; points.points[  214].z = 0.035991;
    points.points[  215].x = 0.025595;  points.points[  215].y = 0.040346; points.points[  215].z = 0.034150;
    points.points[  216].x = 0.038455;  points.points[  216].y = 0.043509; points.points[  216].z = 0.028278;
    points.points[  217].x = 0.032031;  points.points[  217].y = 0.043278; points.points[  217].z = 0.029253;
    points.points[  218].x = 0.036581;  points.points[  218].y = 0.040335; points.points[  218].z = 0.025144;
    points.points[  219].x = 0.030190;  points.points[  219].y = 0.039321; points.points[  219].z = 0.026847;
    points.points[  220].x = 0.059333;  points.points[  220].y = 0.067891; points.points[  220].z = 0.017361;
    points.points[  221].x = 0.046500;  points.points[  221].y = 0.071452; points.points[  221].z = 0.019710;
    points.points[  222].x = 0.059562;  points.points[  222].y = 0.057747; points.points[  222].z = 0.018340;
    points.points[  223].x = 0.055636;  points.points[  223].y = 0.049199; points.points[  223].z = 0.019173;
    points.points[  224].x = 0.050500;  points.points[  224].y = 0.045064; points.points[  224].z = 0.019181;
    points.points[  225].x = 0.023000;  points.points[  225].y = 0.047803; points.points[  225].z = 0.039776;
    points.points[  226].x = 0.022389;  points.points[  226].y = 0.038860; points.points[  226].z = 0.038795;
    points.points[  227].x = -0.019545; points.points[  227].y = 0.093900; points.points[  227].z = 0.052205;
    points.points[  228].x = -0.021462; points.points[  228].y = 0.106180; points.points[  228].z = 0.042059;
    points.points[  229].x = -0.031027; points.points[  229].y = 0.103950; points.points[  229].z = 0.041228;
    points.points[  230].x = -0.022521; points.points[  230].y = 0.097723; points.points[  230].z = 0.045194;
    points.points[  231].x = -0.031858; points.points[  231].y = 0.097026; points.points[  231].z = 0.043878;
    points.points[  232].x = -0.043262; points.points[  232].y = 0.104120; points.points[  232].z = 0.040891;
    points.points[  233].x = -0.052154; points.points[  233].y = 0.104040; points.points[  233].z = 0.040972;
    points.points[  234].x = -0.041875; points.points[  234].y = 0.096944; points.points[  234].z = 0.042424;
    points.points[  235].x = -0.051919; points.points[  235].y = 0.096967; points.points[  235].z = 0.043563;
    points.points[  236].x = -0.021489; points.points[  236].y = 0.086672; points.points[  236].z = 0.054767;
    points.points[  237].x = -0.027000; points.points[  237].y = 0.083087; points.points[  237].z = 0.050284;
    points.points[  238].x = -0.021070; points.points[  238].y = 0.077249; points.points[  238].z = 0.054365;
    points.points[  239].x = -0.026011; points.points[  239].y = 0.089634; points.points[  239].z = 0.048981;
    points.points[  240].x = -0.031893; points.points[  240].y = 0.087035; points.points[  240].z = 0.044169;
    points.points[  241].x = -0.025625; points.points[  241].y = 0.074892; points.points[  241].z = 0.047102;
    points.points[  242].x = -0.031970; points.points[  242].y = 0.076900; points.points[  242].z = 0.042177;
    points.points[  243].x = -0.041824; points.points[  243].y = 0.086954; points.points[  243].z = 0.043295;
    points.points[  244].x = -0.051825; points.points[  244].y = 0.086844; points.points[  244].z = 0.044933;
    points.points[  245].x = -0.041918; points.points[  245].y = 0.076728; points.points[  245].z = 0.042564;
    points.points[  246].x = -0.051849; points.points[  246].y = 0.076877; points.points[  246].z = 0.042992;
    points.points[  247].x = -0.061339; points.points[  247].y = 0.103930; points.points[  247].z = 0.041164;
    points.points[  248].x = -0.072672; points.points[  248].y = 0.109760; points.points[  248].z = 0.044294;
    points.points[  249].x = -0.061784; points.points[  249].y = 0.096825; points.points[  249].z = 0.043327;
    points.points[  250].x = -0.070058; points.points[  250].y = 0.096203; points.points[  250].z = 0.041397;
    points.points[  251].x = -0.080439; points.points[  251].y = 0.110910; points.points[  251].z = 0.044343;
    points.points[  252].x = -0.061927; points.points[  252].y = 0.086724; points.points[  252].z = 0.044520;
    points.points[  253].x = -0.070344; points.points[  253].y = 0.087352; points.points[  253].z = 0.041908;
    points.points[  254].x = -0.061410; points.points[  254].y = 0.077489; points.points[  254].z = 0.042178;
    points.points[  255].x = -0.068579; points.points[  255].y = 0.080144; points.points[  255].z = 0.041024;
    points.points[  256].x = -0.019045; points.points[  256].y = 0.067732; points.points[  256].z = 0.052388;
    points.points[  257].x = -0.017742; points.points[  257].y = 0.058909; points.points[  257].z = 0.050809;
    points.points[  258].x = -0.023548; points.points[  258].y = 0.066382; points.points[  258].z = 0.045226;
    points.points[  259].x = -0.033990; points.points[  259].y = 0.067795; points.points[  259].z = 0.040929;
    points.points[  260].x = -0.021690; points.points[  260].y = 0.056549; points.points[  260].z = 0.045164;
    points.points[  261].x = -0.036111; points.points[  261].y = 0.060706; points.points[  261].z = 0.040407;
    points.points[  262].x = -0.041231; points.points[  262].y = 0.066951; points.points[  262].z = 0.041392;
    points.points[  263].x = -0.048588; points.points[  263].y = 0.070956; points.points[  263].z = 0.040357;
    points.points[  264].x = -0.040300; points.points[  264].y = 0.059465; points.points[  264].z = 0.040446;
    points.points[  265].x = -0.021920; points.points[  265].y = 0.044965; points.points[  265].z = 0.052258;
    points.points[  266].x = -0.029187; points.points[  266].y = 0.043585; points.points[  266].z = 0.051088;
    points.points[  267].x = -0.021919; points.points[  267].y = 0.039826; points.points[  267].z = 0.053521;
    points.points[  268].x = -0.030331; points.points[  268].y = 0.039749; points.points[  268].z = 0.052133;
    points.points[  269].x = -0.021998; points.points[  269].y = 0.049847; points.points[  269].z = 0.046725;
    points.points[  270].x = -0.031911; points.points[  270].y = 0.046848; points.points[  270].z = 0.045187;
    points.points[  271].x = -0.035276; points.points[  271].y = 0.039753; points.points[  271].z = 0.047529;
    points.points[  272].x = -0.042016; points.points[  272].y = 0.044823; points.points[  272].z = 0.041594;
    points.points[  273].x = -0.051940; points.points[  273].y = 0.044707; points.points[  273].z = 0.043498;
    points.points[  274].x = -0.041928; points.points[  274].y = 0.039327; points.points[  274].z = 0.043582;
    points.points[  275].x = -0.051857; points.points[  275].y = 0.039252; points.points[  275].z = 0.046212;
    points.points[  276].x = -0.059453; points.points[  276].y = 0.044240; points.points[  276].z = 0.042862;
    points.points[  277].x = -0.060765; points.points[  277].y = 0.039087; points.points[  277].z = 0.044363;
    points.points[  278].x = -0.024273; points.points[  278].y = 0.110380; points.points[  278].z = 0.039129;
    points.points[  279].x = -0.032379; points.points[  279].y = 0.108780; points.points[  279].z = 0.037952;
    points.points[  280].x = -0.041152; points.points[  280].y = 0.108530; points.points[  280].z = 0.037969;
    points.points[  281].x = -0.051698; points.points[  281].y = 0.109060; points.points[  281].z = 0.038258;
    points.points[  282].x = -0.062091; points.points[  282].y = 0.108770; points.points[  282].z = 0.038274;
    points.points[  283].x = -0.071655; points.points[  283].y = 0.105960; points.points[  283].z = 0.037516;
    points.points[  284].x = -0.074634; points.points[  284].y = 0.097746; points.points[  284].z = 0.038347;
    points.points[  285].x = -0.079120; points.points[  285].y = 0.105080; points.points[  285].z = 0.032308;
    points.points[  286].x = -0.080203; points.points[  286].y = 0.096758; points.points[  286].z = 0.033592;
    points.points[  287].x = -0.083780; points.points[  287].y = 0.105680; points.points[  287].z = 0.025985;
    points.points[  288].x = -0.087292; points.points[  288].y = 0.103140; points.points[  288].z = 0.020825;
    points.points[  289].x = -0.085210; points.points[  289].y = 0.097079; points.points[  289].z = 0.027810;
    points.points[  290].x = -0.088082; points.points[  290].y = 0.096456; points.points[  290].z = 0.022985;
    points.points[  291].x = -0.075160; points.points[  291].y = 0.086040; points.points[  291].z = 0.038816;
    points.points[  292].x = -0.064577; points.points[  292].y = 0.073455; points.points[  292].z = 0.038970;
    points.points[  293].x = -0.072279; points.points[  293].y = 0.076416; points.points[  293].z = 0.036413;
    points.points[  294].x = -0.076375; points.points[  294].y = 0.072563; points.points[  294].z = 0.028730;
    points.points[  295].x = -0.080031; points.points[  295].y = 0.087076; points.points[  295].z = 0.034290;
    points.points[  296].x = -0.078919; points.points[  296].y = 0.079371; points.points[  296].z = 0.032477;
    points.points[  297].x = -0.084834; points.points[  297].y = 0.086686; points.points[  297].z = 0.026974;
    points.points[  298].x = -0.087891; points.points[  298].y = 0.089233; points.points[  298].z = 0.022611;
    points.points[  299].x = -0.081048; points.points[  299].y = 0.077169; points.points[  299].z = 0.025829;
    points.points[  300].x = -0.086393; points.points[  300].y = 0.107840; points.points[  300].z = 0.018635;
    points.points[  301].x = -0.087672; points.points[  301].y = 0.104920; points.points[  301].z = 0.017264;
    points.points[  302].x = -0.089333; points.points[  302].y = 0.098483; points.points[  302].z = 0.017610;
    points.points[  303].x = -0.086375; points.points[  303].y = 0.083067; points.points[  303].z = 0.018607;
    points.points[  304].x = -0.089179; points.points[  304].y = 0.089186; points.points[  304].z = 0.018947;
    points.points[  305].x = -0.082879; points.points[  305].y = 0.076109; points.points[  305].z = 0.017794;
    points.points[  306].x = -0.082500; points.points[  306].y = 0.074674; points.points[  306].z = 0.007118;
    points.points[  307].x = -0.026437; points.points[  307].y = 0.064141; points.points[  307].z = 0.039321;
    points.points[  308].x = -0.030035; points.points[  308].y = 0.066130; points.points[  308].z = 0.038942;
    points.points[  309].x = -0.026131; points.points[  309].y = 0.056531; points.points[  309].z = 0.038882;
    points.points[  310].x = -0.031664; points.points[  310].y = 0.056657; points.points[  310].z = 0.037742;
    points.points[  311].x = -0.045716; points.points[  311].y = 0.064541; points.points[  311].z = 0.039166;
    points.points[  312].x = -0.051959; points.points[  312].y = 0.066869; points.points[  312].z = 0.036733;
    points.points[  313].x = -0.042557; points.points[  313].y = 0.055545; points.points[  313].z = 0.039026;
    points.points[  314].x = -0.049406; points.points[  314].y = 0.056892; points.points[  314].z = 0.034344;
    points.points[  315].x = -0.055500; points.points[  315].y = 0.062391; points.points[  315].z = 0.029498;
    points.points[  316].x = -0.053750; points.points[  316].y = 0.058574; points.points[  316].z = 0.026313;
    points.points[  317].x = -0.034060; points.points[  317].y = 0.050137; points.points[  317].z = 0.038577;
    points.points[  318].x = -0.041741; points.points[  318].y = 0.049590; points.points[  318].z = 0.039290;
    points.points[  319].x = -0.050975; points.points[  319].y = 0.049435; points.points[  319].z = 0.036965;
    points.points[  320].x = -0.053000; points.points[  320].y = 0.051065; points.points[  320].z = 0.029209;
    points.points[  321].x = -0.054145; points.points[  321].y = 0.054568; points.points[  321].z = 0.012257;
    points.points[  322].x = -0.055848; points.points[  322].y = 0.054170; points.points[  322].z = 0.008327;
    points.points[  323].x = -0.054844; points.points[  323].y = 0.049295; points.points[  323].z = 0.011462;
    points.points[  324].x = -0.056150; points.points[  324].y = 0.050619; points.points[  324].z = 0.009293;
    points.points[  325].x = -0.061451; points.points[  325].y = 0.068257; points.points[  325].z = 0.035376;
    points.points[  326].x = -0.069725; points.points[  326].y = 0.069958; points.points[  326].z = 0.032788;
    points.points[  327].x = -0.062823; points.points[  327].y = 0.063322; points.points[  327].z = 0.026886;
    points.points[  328].x = -0.071037; points.points[  328].y = 0.066787; points.points[  328].z = 0.025228;
    points.points[  329].x = -0.060857; points.points[  329].y = 0.060568; points.points[  329].z = 0.022643;
    points.points[  330].x = -0.067000; points.points[  330].y = 0.061558; points.points[  330].z = 0.020109;
    points.points[  331].x = -0.078200; points.points[  331].y = 0.071279; points.points[  331].z = 0.021032;
    points.points[  332].x = -0.062116; points.points[  332].y = 0.045145; points.points[  332].z = 0.037802;
    points.points[  333].x = -0.065473; points.points[  333].y = 0.039513; points.points[  333].z = 0.037964;
    points.points[  334].x = -0.067250; points.points[  334].y = 0.037420; points.points[  334].z = 0.033413;
    points.points[  335].x = -0.072702; points.points[  335].y = 0.065008; points.points[  335].z = 0.018701;
    points.points[  336].x = -0.061450; points.points[  336].y = 0.059165; points.points[  336].z = 0.018731;
    points.points[  337].x = -0.067500; points.points[  337].y = 0.061479; points.points[  337].z = 0.019221;
    points.points[  338].x = -0.057411; points.points[  338].y = 0.054114; points.points[  338].z = 0.003826;
    points.points[  339].x = -0.079222; points.points[  339].y = 0.070654; points.points[  339].z = 0.017735;
    points.points[  340].x = -0.062473; points.points[  340].y = 0.044680; points.points[  340].z = 0.011110;
    points.points[  341].x = -0.067250; points.points[  341].y = 0.042258; points.points[  341].z = 0.010414;
    points.points[  342].x = -0.066389; points.points[  342].y = 0.040515; points.points[  342].z = 0.013160;
    points.points[  343].x = -0.068359; points.points[  343].y = 0.038502; points.points[  343].z = 0.011958;
    points.points[  344].x = -0.061381; points.points[  344].y = 0.047480; points.points[  344].z = 0.007607;
    points.points[  345].x = -0.068559; points.points[  345].y = 0.043549; points.points[  345].z = 0.008158;
    points.points[  346].x = -0.070929; points.points[  346].y = 0.039830; points.points[  346].z = 0.008589;
    points.points[  347].x = -0.016625; points.points[  347].y = 0.183750; points.points[  347].z = -0.019735;
    points.points[  348].x = -0.015198; points.points[  348].y = 0.174710; points.points[  348].z = -0.018868;
    points.points[  349].x = -0.015944; points.points[  349].y = 0.162640; points.points[  349].z = -0.009104;
    points.points[  350].x = -0.015977; points.points[  350].y = 0.160700; points.points[  350].z = -0.008807;
    points.points[  351].x = -0.013251; points.points[  351].y = 0.167080; points.points[  351].z = -0.015264;
    points.points[  352].x = -0.014292; points.points[  352].y = 0.160980; points.points[  352].z = -0.011252;
    points.points[  353].x = -0.013986; points.points[  353].y = 0.184000; points.points[  353].z = -0.023739;
    points.points[  354].x = -0.011633; points.points[  354].y = 0.176990; points.points[  354].z = -0.023349;
    points.points[  355].x = -0.009103; points.points[  355].y = 0.169880; points.points[  355].z = -0.021457;
    points.points[  356].x = -0.025562; points.points[  356].y = 0.182730; points.points[  356].z = -0.009625;
    points.points[  357].x = -0.027250; points.points[  357].y = 0.182540; points.points[  357].z = -0.009438;
    points.points[  358].x = -0.025736; points.points[  358].y = 0.179480; points.points[  358].z = -0.008965;
    points.points[  359].x = -0.031216; points.points[  359].y = 0.175890; points.points[  359].z = -0.005115;
    points.points[  360].x = -0.020399; points.points[  360].y = 0.184500; points.points[  360].z = -0.014943;
    points.points[  361].x = -0.021339; points.points[  361].y = 0.176450; points.points[  361].z = -0.014566;
    points.points[  362].x = -0.027125; points.points[  362].y = 0.172340; points.points[  362].z = -0.010156;
    points.points[  363].x = -0.039390; points.points[  363].y = 0.173300; points.points[  363].z = -0.002357;
    points.points[  364].x = -0.022876; points.points[  364].y = 0.164060; points.points[  364].z = -0.007810;
    points.points[  365].x = -0.031597; points.points[  365].y = 0.166510; points.points[  365].z = -0.004929;
    points.points[  366].x = -0.022600; points.points[  366].y = 0.159120; points.points[  366].z = -0.003799;
    points.points[  367].x = -0.030372; points.points[  367].y = 0.157670; points.points[  367].z = -0.001267;
    points.points[  368].x = -0.021158; points.points[  368].y = 0.168490; points.points[  368].z = -0.012383;
    points.points[  369].x = -0.027000; points.points[  369].y = 0.171200; points.points[  369].z = -0.010220;
    points.points[  370].x = -0.041719; points.points[  370].y = 0.168130; points.points[  370].z = -0.000750;
    points.points[  371].x = -0.048250; points.points[  371].y = 0.167480; points.points[  371].z = -0.000152;
    points.points[  372].x = -0.037250; points.points[  372].y = 0.161470; points.points[  372].z = -0.000073;
    points.points[  373].x = -0.066429; points.points[  373].y = 0.157830; points.points[  373].z = -0.008567;
    points.points[  374].x = -0.071284; points.points[  374].y = 0.158390; points.points[  374].z = -0.005998;
    points.points[  375].x = -0.065979; points.points[  375].y = 0.162880; points.points[  375].z = -0.017792;
    points.points[  376].x = -0.071623; points.points[  376].y = 0.163840; points.points[  376].z = -0.015760;
    points.points[  377].x = -0.066068; points.points[  377].y = 0.160510; points.points[  377].z = -0.013567;
    points.points[  378].x = -0.073307; points.points[  378].y = 0.160490; points.points[  378].z = -0.011832;
    points.points[  379].x = -0.077000; points.points[  379].y = 0.162040; points.points[  379].z = -0.019241;
    points.points[  380].x = -0.077179; points.points[  380].y = 0.158510; points.points[  380].z = -0.014950;
    points.points[  381].x = -0.073691; points.points[  381].y = 0.172860; points.points[  381].z = -0.037944;
    points.points[  382].x = -0.077550; points.points[  382].y = 0.172210; points.points[  382].z = -0.039175;
    points.points[  383].x = -0.065921; points.points[  383].y = 0.165860; points.points[  383].z = -0.025022;
    points.points[  384].x = -0.072095; points.points[  384].y = 0.167840; points.points[  384].z = -0.024725;
    points.points[  385].x = -0.066000; points.points[  385].y = 0.168080; points.points[  385].z = -0.030916;
    points.points[  386].x = -0.073448; points.points[  386].y = 0.170510; points.points[  386].z = -0.032045;
    points.points[  387].x = -0.077770; points.points[  387].y = 0.164340; points.points[  387].z = -0.025938;
    points.points[  388].x = -0.077893; points.points[  388].y = 0.160390; points.points[  388].z = -0.021299;
    points.points[  389].x = -0.078211; points.points[  389].y = 0.169000; points.points[  389].z = -0.034566;
    points.points[  390].x = -0.034667; points.points[  390].y = 0.151310; points.points[  390].z = -0.000710;
    points.points[  391].x = -0.066117; points.points[  391].y = 0.173530; points.points[  391].z = -0.047453;
    points.points[  392].x = -0.071986; points.points[  392].y = 0.176120; points.points[  392].z = -0.045384;
    points.points[  393].x = -0.069250; points.points[  393].y = 0.182000; points.points[  393].z = -0.055026;
    points.points[  394].x = -0.064992; points.points[  394].y = 0.178020; points.points[  394].z = -0.054645;
    points.points[  395].x = -0.069935; points.points[  395].y = 0.179830; points.points[  395].z = -0.051988;
    points.points[  396].x = -0.077930; points.points[  396].y = 0.175160; points.points[  396].z = -0.044400;
  }
}

#endif
