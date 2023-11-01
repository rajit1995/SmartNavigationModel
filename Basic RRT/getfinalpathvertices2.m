function RRTState1 = getfinalpathvertices2(RRTState1)
RRTState1.finalpathvertices = [];
for i = 1:size(RRTState1.path,2)
RRTState1.finalpathvertices(i,:) = RRTState1.pathvertices(RRTState1.path(1,i),:);
end

end