% CAMERA2IMAGE

%  Copyright (c) 2011 DFKI GmbH
%  All rights reserved
%
%  Author: Oliver Birbach <oliver.birbach@dfki.de>
%
%  Redistribution and use in source and binary forms, with or without
%  modification, are permitted provided that the following conditions
%  are met:
%
%   * Redistributions of source code must retain the above copyright
%     notice, this list of conditions and the following disclaimer.
%   * Redistributions in binary form must reproduce the above
%     copyright notice, this list of conditions and the following
%     disclaimer in the documentation and/or other materials provided
%     with the distribution.
%   * Neither the name of DFKI GmbH nor the names of any
%     contributors may be used to endorse or promote products derived
%     from this software without specific prior written permission.
%
%  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
%  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
%  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
%  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
%  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
%  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
%  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
%  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
%  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
%  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
%  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
%  POSSIBILITY OF SUCH DAMAGE.
%

function imagepoint = camera2Image(v, k1, k2, k3, k4, k5, k6, p1, p2, scale, offset)
% Projects vector v from camera space to the image plane. v has to be
% m x 3 matrix containing the m vectors to be transformed.

    % Perspective
    xp = (v(:,1) ./ v(:,3));
    yp = (v(:,2) ./ v(:,3));
    
    % Distortion
    r2 = xp.^2 + yp.^2;
    r4 = xp.^4 + yp.^4;
    r6 = xp.^6 + yp.^6;
    nom = 1 + k1.*r2 + k2.*r4 + k3.*r6;
    denom = 1+ k4.*r2 + k5.*r4 + k6.*r6;
    
    xpp = xp .* nom/denom + 2*p1.*xp.*yp + p2.*(r2 + 2.*xp.^2);
    ypp = yp .* nom/denom + 2*p2.*xp.*yp + p1.*(r2 + 2.*yp.^2);
    
    imagepoint = [xpp ypp]; 
    
    % Scaling and offset
    imagepoint = [(scale(1) .* imagepoint(:,1) + offset(1)) (scale(2) .* imagepoint(:,2) + offset(2))];