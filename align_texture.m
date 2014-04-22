function [texture]= align_texture(texture,shift_step)
    %add filter for back texture
    
    [t_sm,t_sn,~]=size(texture);
    
    avr_r=sum(sum(texture(:,:,1)))/(t_sm*t_sn);
    avr_g=sum(sum(texture(:,:,2)))/(t_sm*t_sn);
    avr_b=sum(sum(texture(:,:,3)))/(t_sm*t_sn);
    
    %customized back texture
    tmp=ones(t_sm,t_sn,3);
    tmp(:,:,1)=tmp(:,:,1)*avr_r;
    tmp(:,:,2)=tmp(:,:,2)*avr_g;
    tmp(:,:,3)=tmp(:,:,3)*avr_b;
    
    
    if(shift_step>=0 &&shift_step<=7)
        pieces=t_sn/8;
        texture = [  texture(:,t_sn-shift_step*pieces:t_sn,:) tmp texture(:,1:t_sn-shift_step*pieces,:) ];
    end
    
    if(shift_step<0 &&shift_step>=-7)
         shift_step=-shift_step;
         texture = [ tmp(:,1:round(shift_step* t_sn/8),:) texture tmp(:,t_sn-round(shift_step* t_sn/8):t_sn,:)];
    end
end
