function [ out_im ] = Bill_get_out_imgs( positions, imgs, height_top, height_bottom, width )

    num_frames = numel(imgs(1,1,:));
    out_im = uint8(zeros(height_top + height_bottom, width, num_frames));
    for frame = 1:num_frames
        x1 = round(positions(frame, 2) - width/2);
        x2 = round(positions(frame, 2) + width/2 - 1);
        y1 = round(positions(frame, 1) - height_top);
        y2 = round(positions(frame, 1) + height_bottom - 1);

        out_im(:, :, frame) = imgs(y1:y2, x1:x2, frame);
    end
end

