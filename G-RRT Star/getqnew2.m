function RRTState1 = getqnew2(RRTState1)
    if norm(RRTState1.q_near - RRTState1.q_new) <= RRTState1.StepSize
        RRTState1.q_new = RRTState1.q_new;
    else
        RRTState1.q_new = RRTState1.q_near + RRTState1.StepSize*(RRTState1.q_new - RRTState1.q_near)/norm((RRTState1.q_new - RRTState1.q_near));
        
    end   
end
