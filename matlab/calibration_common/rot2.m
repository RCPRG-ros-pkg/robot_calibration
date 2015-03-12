function [ M ] = rot2(angle, rotvec)
%ROT2 Summary of this function goes here
%   Detailed explanation goes here
    ct = cos(angle);
    st = sin(angle);
    vt = 1-ct;
    m_vt_0=vt*rotvec(1);
    m_vt_1=vt*rotvec(2);
    m_vt_2=vt*rotvec(3);
    m_st_0=rotvec(1)*st;
    m_st_1=rotvec(2)*st;
    m_st_2=rotvec(3)*st;
    m_vt_0_1=m_vt_0*rotvec(2);
    m_vt_0_2=m_vt_0*rotvec(3);
    m_vt_1_2=m_vt_1*rotvec(3);

    M = [ct + m_vt_0*rotvec(1), -m_st_2 + m_vt_0_1, m_st_1 + m_vt_0_2;
        m_st_2 + m_vt_0_1, ct + m_vt_1*rotvec(2), -m_st_0 + m_vt_1_2;
        -m_st_1 + m_vt_0_2, m_st_0 + m_vt_1_2, ct + m_vt_2*rotvec(3)];
end

