clear;
syms x yaw alphal alphar theta;
syms dx dyaw dalphal dalphar dtheta;
syms ddx ddyaw ddalphal ddalphar ddtheta;
syms Tl Tpl Tr Tpr;

syms mw mp m;
syms Iw Ip Im;
syms L Lm l d;
syms g R;

ddx = ((Tl+Tr)-(Nl+Nr)*R)/(2*(mw*R+Iw*R));
