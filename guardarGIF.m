function guardarGIF(gifFileName, frame, pauseTime, isFirstFrame)
    [imind, cm] = rgb2ind(frame2im(frame), 256);

    if isFirstFrame
        imwrite(imind, cm, gifFileName, 'gif', 'Loopcount', inf, 'DelayTime', pauseTime);
    else
        imwrite(imind, cm, gifFileName, 'gif', 'WriteMode', 'append', 'DelayTime', pauseTime);
    end

    pause(pauseTime);
end