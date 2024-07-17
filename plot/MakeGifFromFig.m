function MakeGifFromFig(gifname, figname, index)  
    imind = imread(figname);  
    [imind,cm] = rgb2ind(imind, 256);  
    if index==1  
        imwrite(imind,cm,gifname, 'Loopcount',inf,'DelayTime',0.001);
    else  
        imwrite(imind,cm,gifname, 'WriteMode','append','DelayTime',0.001);
    end  
end  