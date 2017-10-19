% This is PUMA3d.M, a 3D Matlab Kinematic model of a Puma robot located
% in the robotics lab of Walla Walla University.
% The file uses CAD data converted to Matlab using cad2matdemo.m, which 
% is located on the Mathworks central file exchange.
%
% This file is still being developed, for the latest version check the
% Mathworks central file exchange.
%
% Todo list:
% 1) optimize pumaANI, lots of stuff in loop that needs help.
% 2) move x, y, and z position to end effecter, not link 6 origin.
% 3) Toggle kinematics buttons on/off with inverse kinematics button.
% 4) Make this work with real time inverse kinematics.
% 5) Make the track on and off option better.
% 6) add other things that makes this program fun.
% 7) check for noplots and nogos
% 8) add some better "demos" for the button
% 9) Fix problem of more than one robot window.
%
function puma3d
% GUI kinematic demo for the Puma Robot.
% Robot geometry uses the CAD2MATDEMO code in the Mathworks file exchange
%
%%
loaddata
InitHome
%
% Create the push buttons: pos is: [left bottom width height]
demo = uicontrol(fig_1,'String','Demo','callback',@demo_button_press,...
    'Position',[20 5 60 20]);

rnd_demo = uicontrol(fig_1,'String','Random Move','callback',@rnd_demo_button_press,...
    'Position',[100 5 80 20]);

clr_trail = uicontrol(fig_1,'String','Clr Trail','callback',@clr_trail_button_press,...
    'Position',[200 5 60 20]);
%
home = uicontrol(fig_1,'String','Home','callback',@home_button_press,...
    'Position',[280 5 70 20]);
%
% Kinematics Panel
%
K_p = uipanel(fig_1,...
    'units','pixels',...
    'Position',[20 45 265 200],...
    'Title','Kinematics','FontSize',11);
%
%     Angle    Range                Default Name
%     Theta 1: 320 (-160 to 160)    90       Waist Joint  
%     Theta 2: 220 (-110 to 110)   -90       Shoulder Joint
%     Theta 3: 270 (-135 to 135)   -90       Elbow Joint    
%     Theta 4: 532 (-266 to 266)     0       Wrist Roll
%     Theta 5: 200 (-100 to 100)     0       Wrist Bend  
%     Theta 6: 532 (-266 to 266)     0       Wrist Swivel

t1_home = 90; % offset to define the "home" position as UP.
t2_home = -90;
t3_home = -90;

LD = 105; % Left, used to set the GUI.
HT = 18;  % Height
BT = 156; % Bottom
%%  GUI buttons for Theta 1.  pos is: [left bottom width height]
t1_slider = uicontrol(K_p,'style','slider',...
    'Max',160,'Min',-160,'Value',0,...
    'SliderStep',[0.05 0.2],...
    'callback',@t1_slider_button_press,...
    'Position',[LD BT 120 HT]);
t1_min = uicontrol(K_p,'style','text',...
    'String','-160',...
    'Position',[LD-30 BT+1 25 HT-4]); % L, from bottom, W, H
t1_max = uicontrol(K_p,'style','text',...
    'String','+160',...
    'Position',[LD+125 BT+1 25 HT-4]); % L, B, W, H
t1_text = uibutton(K_p,'style','text',...  % Nice program Doug. Need this
    'String','\theta_1',...                % due to no TeX in uicontrols. 
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
% t1_text = uicontrol(K_p,'style','text',... % when matlab fixes uicontrol
%     'String','t1',...                      % for TeX, then I can use this.  
%     'Position',[LD-100 BT 20 HT]); % L, B, W, H
t1_edit = uicontrol(K_p,'style','edit',...
    'String',0,...
    'callback',@t1_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H
%
%%  GUI buttons for Theta 2.
BT = 126;   % Bottom
t2_slider = uicontrol(K_p,'style','slider',...
    'Max',115,'Min',-115,'Value',0,...        % Mech. stop limits !
    'SliderStep',[0.05 0.2],...
    'callback',@t2_slider_button_press,...
    'Position',[LD BT 120 HT]);
t2_min = uicontrol(K_p,'style','text',...
    'String','-110',...
    'Position',[LD-30 BT+1 25 HT-4]); % L, from bottom, W, H
t2_max = uicontrol(K_p,'style','text',...
    'String','+110',...
    'Position',[LD+125 BT+1 25 HT-4]); % L, B, W, H
t2_text = uibutton(K_p,'style','text',...
    'String','\theta_2',...
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
t2_edit = uicontrol(K_p,'style','edit',...
    'String',0,...
    'callback',@t2_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H
%
%%  GUI buttons for Theta 3.
BT = 96;   % Bottom
t3_slider = uicontrol(K_p,'style','slider',...
    'Max',135,'Min',-135,'Value',0,...
    'SliderStep',[0.05 0.2],...
    'callback',@t3_slider_button_press,...
    'Position',[LD BT 120 HT]);
t3_min = uicontrol(K_p,'style','text',...
    'String','-135',...
    'Position',[LD-30 BT+1 25 HT-4]); % L, from bottom, W, H
t3_max = uicontrol(K_p,'style','text',...
    'String','+135',...
    'Position',[LD+125 BT+1 25 HT-4]); % L, B, W, H
t3_text = uibutton(K_p,'style','text',...
    'String','\theta_3',...
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
t3_edit = uicontrol(K_p,'style','edit',...
    'String',0,...
    'callback',@t3_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H
%
%%  GUI buttons for Theta 4.
BT = 66;   % Bottom
t4_slider = uicontrol(K_p,'style','slider',...
    'Max',266,'Min',-266,'Value',0,...
    'SliderStep',[0.05 0.2],...
    'callback',@t4_slider_button_press,...
    'Position',[LD BT 120 HT]);
t4_min = uicontrol(K_p,'style','text',...
    'String','-266',...
    'Position',[LD-30 BT+1 25 HT-4]); % L, from bottom, W, H
t4_max = uicontrol(K_p,'style','text',...
    'String','+266',...
    'Position',[LD+125 BT+1 25 HT-4]); % L, B, W, H
t4_text = uibutton(K_p,'style','text',...
    'String','\theta_4',...
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
t4_edit = uicontrol(K_p,'style','edit',...
    'String',0,...
    'callback',@t4_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H
%
%%  GUI buttons for Theta 5.
BT = 36;   % Bottom
t5_slider = uicontrol(K_p,'style','slider',...
    'Max',100,'Min',-100,'Value',0,...
    'SliderStep',[0.05 0.2],...
    'callback',@t5_slider_button_press,...
    'Position',[LD BT 120 HT]);
t5_min = uicontrol(K_p,'style','text',...
    'String','-100',...
    'Position',[LD-30 BT+1 25 HT-4]); % L, from bottom, W, H
t5_max = uicontrol(K_p,'style','text',...
    'String','+100',...
    'Position',[LD+125 BT+1 25 HT-4]); % L, B, W, H
t5_text = uibutton(K_p,'style','text',...
    'String','\theta_5',...
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
t5_edit = uicontrol(K_p,'style','edit',...
    'String',0,...
    'callback',@t5_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H
%
%%  GUI buttons for Theta 6.
BT = 6;   % Bottom
t6_slider = uicontrol(K_p,'style','slider',...
    'Max',266,'Min',-266,'Value',0,...
    'SliderStep',[0.05 0.2],...
    'callback',@t6_slider_button_press,...
    'Position',[LD BT 120 HT]);
t6_min = uicontrol(K_p,'style','text',...
    'String','-266',...
    'Position',[LD-30 BT+1 25 HT-4]); % L, from bottom, W, H
t6_max = uicontrol(K_p,'style','text',...
    'String','+266',...
    'Position',[LD+125 BT+1 25 HT-4]); % L, B, W, H
t6_text = uibutton(K_p,'style','text',...
    'String','\theta_6',...
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
t6_edit = uicontrol(K_p,'style','edit',...
    'String',0,...
    'callback',@t6_edit_button_press,...
    'Position',[LD-75 BT 30 HT]); % L, B, W, H
%
%% Slider for Theta 1 motion.
%
    function t1_slider_button_press(h,dummy)
        slider_value = round(get(h,'Value'));
        set(t1_edit,'string',slider_value);
        T_Old = getappdata(0,'ThetaOld');
        t2old = T_Old(2); t3old = T_Old(3); t4old = T_Old(4);
        t5old = T_Old(5); t6old = T_Old(6);
        pumaANI(slider_value+t1_home,t2old,t3old,t4old,t5old,t6old,10,'n')
    end
%
%% Slider for Theta 2 motion.
%
    function t2_slider_button_press(h,dummy)
        slider_value = round(get(h,'Value'));
        set(t2_edit,'string',slider_value);
        T_Old = getappdata(0,'ThetaOld');
        t1old = T_Old(1); t3old = T_Old(3); t4old = T_Old(4);
        t5old = T_Old(5); t6old = T_Old(6);
        pumaANI(t1old,slider_value+t2_home,t3old,t4old,t5old,t6old,10,'n')
    end
%
%% Slider for Theta 3 motion.
    function t3_slider_button_press(h,dummy)
        slider_value = round(get(h,'Value'));
        set(t3_edit,'string',slider_value);
        T_Old = getappdata(0,'ThetaOld');
        t1old = T_Old(1); t2old = T_Old(2); t4old = T_Old(4);
        t5old = T_Old(5); t6old = T_Old(6);
        pumaANI(t1old,t2old,slider_value+t3_home,t4old,t5old,t6old,10,'n')
    end
%
%% Slider for Theta 4 motion.
    function t4_slider_button_press(h,dummy)
        slider_value = round(get(h,'Value'));
        set(t4_edit,'string',slider_value);
        T_Old = getappdata(0,'ThetaOld');
        t1old = T_Old(1); t2old = T_Old(2); t3old = T_Old(3);
        t5old = T_Old(5); t6old = T_Old(6);
        pumaANI(t1old,t2old,t3old,slider_value,t5old,t6old,10,'n')
    end
%
%% Slider for Theta 5 motion.
    function t5_slider_button_press(h,dummy)
        slider_value = round(get(h,'Value'));
        set(t5_edit,'string',slider_value);
        T_Old = getappdata(0,'ThetaOld');
        t1old = T_Old(1); t2old = T_Old(2); t3old = T_Old(3);
        t4old = T_Old(4); t6old = T_Old(6);
        pumaANI(t1old,t2old,t3old,t4old,slider_value,t6old,10,'n')
    end
%
%% Slider for Theta 6 motion.
    function t6_slider_button_press(h,dummy)
        slider_value = round(get(h,'Value'));
        set(t6_edit,'string',slider_value);
        T_Old = getappdata(0,'ThetaOld');
        t1old = T_Old(1); t2old = T_Old(2); t3old = T_Old(3);
        t4old = T_Old(4); t5old = T_Old(5);
        pumaANI(t1old,t2old,t3old,t4old,t5old,slider_value,10,'n')
    end
%
%% Edit box for Theta 1 motion.
%
    function t1_edit_button_press(h,dummy)
        user_entry = check_edit(h,-160,160,0,t1_edit);
        set(t1_slider,'Value',user_entry);  % slider = text box.
        T_Old = getappdata(0,'ThetaOld');   % Current pose    
        %
        t2old = T_Old(2); t3old = T_Old(3); t4old = T_Old(4);
        t5old = T_Old(5); t6old = T_Old(6);
        %
        pumaANI(user_entry+t1_home,t2old,t3old,t4old,t5old,t6old,10,'n')
    end
%
%% Edit box for Theta 2 motion.
%
    function t2_edit_button_press(h,dummy)
        user_entry = check_edit(h,-110,110,0,t2_edit);
        set(t2_slider,'Value',user_entry);  % slider = text box.
        T_Old = getappdata(0,'ThetaOld');   % Current pose    
        %
        t1old = T_Old(1); t3old = T_Old(3); t4old = T_Old(4);
        t5old = T_Old(5); t6old = T_Old(6);
        %
        pumaANI(t1old,user_entry+t2_home,t3old,t4old,t5old,t6old,10,'n')
    end
%% Edit box for Theta 3 motion.
%
    function t3_edit_button_press(h,dummy)
        user_entry = check_edit(h,-135,135,0,t3_edit);
        set(t3_slider,'Value',user_entry);  % slider = text box.
        T_Old = getappdata(0,'ThetaOld');   % Current pose    
        %
        t1old = T_Old(1); t2old = T_Old(2); t4old = T_Old(4);
        t5old = T_Old(5); t6old = T_Old(6);
        %
        pumaANI(t1old,t2old,user_entry+t3_home,t4old,t5old,t6old,10,'n')
    end
%%
%% Edit box for Theta 4 motion.
%
    function t4_edit_button_press(h,dummy)
        user_entry = check_edit(h,-266,266,0,t4_edit);
        set(t4_slider,'Value',user_entry);  % slider = text box.
        T_Old = getappdata(0,'ThetaOld');   % Current pose    
        %
        t1old = T_Old(1); t2old = T_Old(2); t3old = T_Old(3);
        t5old = T_Old(5); t6old = T_Old(6);
        %
        pumaANI(t1old,t2old,t3old,user_entry,t5old,t6old,10,'n')
    end
%% Edit box for Theta 5 motion.
%
    function t5_edit_button_press(h,dummy)
        user_entry = check_edit(h,-100,100,0,t5_edit);
        set(t5_slider,'Value',user_entry);  % slider = text box.
        T_Old = getappdata(0,'ThetaOld');   % Current pose    
        %
        t1old = T_Old(1); t2old = T_Old(2); t3old = T_Old(3);
        t4old = T_Old(4); t6old = T_Old(6);
        %
        pumaANI(t1old,t2old,t3old,t4old,user_entry,t6old,10,'n')
    end
%%
%% Edit box for Theta 6 motion.
%
    function t6_edit_button_press(h,dummy)
        user_entry = check_edit(h,-266,266,0,t6_edit);
        set(t6_slider,'Value',user_entry);  % slider = text box.
        T_Old = getappdata(0,'ThetaOld');   % Current pose    
        %
        t1old = T_Old(1); t2old = T_Old(2); t3old = T_Old(3);
        t4old = T_Old(4); t5old = T_Old(5);
        %
        pumaANI(t1old,t2old,t3old,t4old,t5old,user_entry,10,'n')
    end
%%
    function user_entry = check_edit(h,min_v,max_v,default,h_edit)
        % This function will check the value typed in the text input box
        % against min and max values, and correct errors.
        %
        % h: handle of gui
        % min_v min value to check
        % max_v max value to check
        % default is the default value if user enters non number
        % h_edit is the edit value to update.
        %
        user_entry = str2double(get(h,'string'));
        if isnan(user_entry)
            errordlg(['You must enter a numeric value, defaulting to ',num2str(default),'.'],'Bad Input','modal')
            set(h_edit,'string',default);
            user_entry = default;
        end
        %
        if user_entry < min_v
            errordlg(['Minimum limit is ',num2str(min_v),' degrees, using ',num2str(min_v),'.'],'Bad Input','modal')
            user_entry = min_v;
            set(h_edit,'string',user_entry);
        end
        if user_entry > max_v
            errordlg(['Maximum limit is ',num2str(max_v),' degrees, using ',num2str(max_v),'.'],'Bad Input','modal')
            user_entry = max_v;
            set(h_edit,'string',user_entry);
        end
    end
%
%% Demo button's callback
    function demo_button_press(h,dummy)
        %
        % disp('pushed demo bottom');
        %         R = 500;
        %         x = 1000;
        n = 2;    % demo ani steps
        num = 30; % home to start, and end to home ani steps
        %         j = 1;
        %         M = 1000;
        for t = 0:.1:7*pi
            Px = 30*t*cos(t);
            Py = 1200-300*t*(t)/(50*pi);
            Pz = 30*t*sin(t);
            [theta1,theta2,theta3,theta4,theta5,theta6] = PumaIK(Px,Py,Pz);
            if t==0 %move to start of demo
                pumaANI(theta1,theta2,theta3-180,0,0,0,num,'n')
            end
            % Theta 4, 5 & 6 are zero due to plotting at wrist origen.
            pumaANI(theta1,theta2,theta3-180,0,0,0,n,'y')
            set(t1_edit,'string',round(theta1)); % Update slider and text.
            set(t1_slider,'Value',round(theta1));
            set(t2_edit,'string',round(theta2));
            set(t2_slider,'Value',round(theta2));
            set(t3_edit,'string',round(theta3-180));
            set(t3_slider,'Value',round(theta3-180));
        end
        gohome
%        pumaANI(90,-90,-90,0,0,0,num,'n')
    end
%
%
%%
    function home_button_press(h,dummy)
        %disp('pushed home bottom');
        gohome
    end
%
%%
    function clr_trail_button_press(h,dummy)
        %disp('pushed clear trail bottom');
        handles = getappdata(0,'patch_h');           %
        Tr = handles(9);
        %
        setappdata(0,'xtrail',0); % used for trail tracking.
        setappdata(0,'ytrail',0); % used for trail tracking.
        setappdata(0,'ztrail',0); % used for trail tracking.
        %
        set(Tr,'xdata',0,'ydata',0,'zdata',0);
    end
%
%
    function rnd_demo_button_press(h, dummy)
        %disp('pushed random demo bottom');
        % a = 10; b = 50; x = a + (b-a) * rand(5)
        %     Angle    Range                Default Name
        %     Theta 1: 320 (-160 to 160)    90       Waist Joint
        %     Theta 2: 220 (-110 to 110)    -90       Shoulder Joint
        %     Theta 3: 270 (-135 to 135)    -90       Elbow Joint
        %     Theta 4: 532 (-266 to 266)    0       Wrist Roll
        %     Theta 5: 200 (-100 to 100)    0       Wrist Bend
        %     Theta 6: 532 (-266 to 266)    0       Wrist Swival
        t1_home = 90; % offsets to define the "home" postition as UP.
        t2_home = -90;
        t3_home = -90;
        theta1 = -160 + 320*rand(1); % offset for home
        theta2 = -110 + 220*rand(1); % in the UP pos.
        theta3 = -135 + 270*rand(1);
        theta4 = -266 + 532*rand(1);
        theta5 = -100 + 200*rand(1);
        theta6 = -266 + 532*rand(1);
        n = 50;
        pumaANI(theta1+t1_home,theta2+t2_home,theta3+t3_home,theta4,theta5,theta6,n,'y')
        set(t1_edit,'string',round(theta1)); % Update slider and text.
        set(t1_slider,'Value',round(theta1));
        set(t2_edit,'string',round(theta2));
        set(t2_slider,'Value',round(theta2));
        set(t3_edit,'string',round(theta3));
        set(t3_slider,'Value',round(theta3));
        set(t4_edit,'string',round(theta4));
        set(t4_slider,'Value',round(theta4));
        set(t5_edit,'string',round(theta5));
        set(t5_slider,'Value',round(theta5));
        set(t6_edit,'string',round(theta6));
        set(t6_slider,'Value',round(theta6));
    end
        


%%
%Here are the functions used for this robot example:
%
%%
% When called this function will simply initialize a plot of the Puma 762
% robot by plotting it in it's home orientation and setting the current
% angles accordingly.
    function gohome()
        pumaANI(90,-90,-90,0,0,0,20,'n') % show it animate home
        %PumaPOS(90,-90,-90,0,0,0)  %drive it home, no animate.
        set(t1_edit,'string',0);
        set(t1_slider,'Value',0);  %At the home position, so all
        set(t2_edit,'string',0);   %sliders and input boxes = 0. 
        set(t2_slider,'Value',0);
        set(t3_edit,'string',0);
        set(t3_slider,'Value',0);
        set(t4_edit,'string',0);
        set(t4_slider,'Value',0);
        set(t5_edit,'string',0);
        set(t5_slider,'Value',0);
        set(t6_edit,'string',0);
        set(t6_slider,'Value',0);
        setappdata(0,'ThetaOld',[90,-90,-90,0,0,0]);
    end
%%
% This function will load the 3D CAD data.
%
function loaddata
% Loads all the link data from file linksdata.mat.
% This data comes from a Pro/E 3D CAD model and was made with cad2matdemo.m
% from the file exchange.  All link data manually stored in linksdata.mat
[linkdata]=load('linksdata.mat','s1','s2', 's3','s4','s5','s6','s7','A1');

%Place the robot link 'data' in a storage area
setappdata(0,'Link1_data',linkdata.s1);
setappdata(0,'Link2_data',linkdata.s2);
setappdata(0,'Link3_data',linkdata.s3);
setappdata(0,'Link4_data',linkdata.s4);
setappdata(0,'Link5_data',linkdata.s5);
setappdata(0,'Link6_data',linkdata.s6);
setappdata(0,'Link7_data',linkdata.s7);
setappdata(0,'Area_data',linkdata.A1);
end
%
%%
% Use forward kinematics to place the robot in a specified configuration.
%
    function PumaPOS(theta1,theta2,theta3,theta4,theta5,theta6)

        s1 = getappdata(0,'Link1_data');
        s2 = getappdata(0,'Link2_data');
        s3 = getappdata(0,'Link3_data');
        s4 = getappdata(0,'Link4_data');
        s5 = getappdata(0,'Link5_data');
        s6 = getappdata(0,'Link6_data');
        s7 = getappdata(0,'Link7_data');
        A1 = getappdata(0,'Area_data');
        %
        a2 = 650;
        a3 = 0;
        d3 = 190;
        d4 = 600;
        Px = 5000;
        Py = 5000;
        Pz = 5000;

        t1 = theta1; 
        t2 = theta2; 
        t3 = theta3 %-180;  
        t4 = theta4; 
        t5 = theta5; 
        t6 = theta6; 
        %
        % Forward Kinematics
        T_01 = tmat(0, 0, 0, t1);
        T_12 = tmat(-90, 0, 0, t2);
        T_23 = tmat(0, a2, d3, t3);
        T_34 = tmat(-90, a3, d4, t4);
        T_45 = tmat(90, 0, 0, t5);
        T_56 = tmat(-90, 0, 0, t6);

        %T_01 = T_01;
        T_02 = T_01*T_12;
        T_03 = T_02*T_23;
        T_04 = T_03*T_34;
        T_05 = T_04*T_45;
        T_06 = T_05*T_56;
        %
        Link1 = s1.V1;
        Link2 = (T_01*s2.V2')';
        Link3 = (T_02*s3.V3')';
        Link4 = (T_03*s4.V4')';
        Link5 = (T_04*s5.V5')';
        Link6 = (T_05*s6.V6')';
        Link7 = (T_06*s7.V7')';

        handles = getappdata(0,'patch_h');           %
        L1 = handles(1);
        L2 = handles(2);
        L3 = handles(3);
        L4 = handles(4);
        L5 = handles(5);
        L6 = handles(6);
        L7 = handles(7);
        %
        set(L1,'vertices',Link1(:,1:3),'facec', [0.717,0.116,0.123]);
        set(L1, 'EdgeColor','none');
        set(L2,'vertices',Link2(:,1:3),'facec', [0.216,1,.583]);
        set(L2, 'EdgeColor','none');
        set(L3,'vertices',Link3(:,1:3),'facec', [0.306,0.733,1]);
        set(L3, 'EdgeColor','none');
        set(L4,'vertices',Link4(:,1:3),'facec', [1,0.542,0.493]);
        set(L4, 'EdgeColor','none');
        set(L5,'vertices',Link5(:,1:3),'facec', [0.216,1,.583]);
        set(L5, 'EdgeColor','none');
        set(L6,'vertices',Link6(:,1:3),'facec', [1,1,0.255]);
        set(L6, 'EdgeColor','none');
        set(L7,'vertices',Link7(:,1:3),'facec', [0.306,0.733,1]);
        set(L7, 'EdgeColor','none');
    end
%%
% This function computes the Inverse Kinematics for the Puma 762 robot
% given X,Y,Z coordinates for a point in the workspace. Note: The IK are
% computed for the origin of Coordinate systems 4,5 & 6.
    function [theta1,theta2,theta3,theta4,theta5,theta6] = PumaIK(Px,Py,Pz)
        theta4 = 0;
        theta5 = 0;
        theta6 = 0;
        sign1 = 1;
        sign3 = 1;
        nogo = 0;
        noplot = 0;
        % Because the sqrt term in theta1 & theta3 can be + or - we run through
        % all possible combinations (i = 4) and take the first combination that
        % satisfies the joint angle constraints.
        while nogo == 0;
            for i = 1:1:4
                if i == 1
                    sign1 = 1;
                    sign3 = 1;
                elseif i == 2
                    sign1 = 1;
                    sign3 = -1;
                elseif i == 3
                    sign1 = -1;
                    sign3 = 1;
                else
                    sign1 = -1;
                    sign3 = -1;
                end
                a2 = 650;
                a3 = 0;
                d3 = 190;
                d4 = 600;
                rho = sqrt(Px^2+Py^2);
                phi = atan2(Py,Px);
                K = (Px^2+Py^2+Pz^2-a2^2-a3^2-d3^2-d4^2)/(2*a2);
                c4 = cos(theta4);
                s4 = sin(theta4);
                c5 = cos(theta5);
                s5 = sin(theta5);
                c6 = cos(theta6);
                s6 = sin(theta6);
                theta1 = (atan2(Py,Px)-atan2(d3,sign1*sqrt(Px^2+Py^2-d3^2)));

                c1 = cos(theta1);
                s1 = sin(theta1);
                theta3 = (atan2(a3,d4)-atan2(K,sign3*sqrt(a3^2+d4^2-K^2)));

                c3 = cos(theta3);
                s3 = sin(theta3);
                t23 = atan2((-a3-a2*c3)*Pz-(c1*Px+s1*Py)*(d4-a2*s3),(a2*s3-d4)*Pz+(a3+a2*c3)*(c1*Px+s1*Py));
                theta2 = (t23 - theta3);

                c2 = cos(theta2);
                s2 = sin(theta2);
                s23 = ((-a3-a2*c3)*Pz+(c1*Px+s1*Py)*(a2*s3-d4))/(Pz^2+(c1*Px+s1*Py)^2);
                c23 = ((a2*s3-d4)*Pz+(a3+a2*c3)*(c1*Px+s1*Py))/(Pz^2+(c1*Px+s1*Py)^2);
                r13 = -c1*(c23*c4*s5+s23*c5)-s1*s4*s5;
                r23 = -s1*(c23*c4*s5+s23*c5)+c1*s4*s5;
                r33 = s23*c4*s5 - c23*c5;
                theta4 = atan2(-r13*s1+r23*c1,-r13*c1*c23-r23*s1*c23+r33*s23);

                r11 = c1*(c23*(c4*c5*c6-s4*s6)-s23*s5*c6)+s1*(s4*c5*c6+c4*s6);
                r21 = s1*(c23*(c4*c5*c6-s4*s6)-s23*s5*c6)-c1*(s4*c5*c6+c4*s6);
                r31 = -s23*(c4*c5*c6-s4*s6)-c23*s5*c6;
                s5 = -(r13*(c1*c23*c4+s1*s4)+r23*(s1*c23*c4-c1*s4)-r33*(s23*c4));
                c5 = r13*(-c1*s23)+r23*(-s1*s23)+r33*(-c23);
                theta5 = atan2(s5,c5);

                s6 = -r11*(c1*c23*s4-s1*c4)-r21*(s1*c23*s4+c1*c4)+r31*(s23*s4);
                c6 = r11*((c1*c23*c4+s1*s4)*c5-c1*s23*s5)+r21*((s1*c23*c4-c1*s4)*c5-s1*s23*s5)-r31*(s23*c4*c5+c23*s5);
                theta6 = atan2(s6,c6);

                theta1 = theta1*180/pi;
                theta2 = theta2*180/pi;
                theta3 = theta3*180/pi;
                theta4 = theta4*180/pi;
                theta5 = theta5*180/pi;
                theta6 = theta6*180/pi;
                if theta2>=160 && theta2<=180
                    theta2 = -theta2;
                end

                if theta1<=160 && theta1>=-160 && (theta2<=20 && theta2>=-200) && theta3<=45 && theta3>=-225 && theta4<=266 && theta4>=-266 && theta5<=100 && theta5>=-100 && theta6<=266 && theta6>=-266
                    nogo = 1;
                    theta3 = theta3+180;
                    break
                end
                if i == 4 && nogo == 0
                    h = errordlg('Point unreachable due to joint angle constraints.','JOINT ERROR');
                    waitfor(h);
                    nogo = 1;
                    noplot = 1;
                    break
                end
            end
        end
    end
%
%% 
    function pumaANI(theta1,theta2,theta3,theta4,theta5,theta6,n,trail)
        % This function will animate the Puma 762 robot given joint angles.
        % n is number of steps for the animation
        % trail is 'y' or 'n' (n = anything else) for leaving a trail.
        %
        %disp('in animate');
        a2 = 650; %D-H paramaters
        a3 = 0;
        d3 = 190;
        d4 = 600;
        % Err2 = 0;
        %
        ThetaOld = getappdata(0,'ThetaOld');
        %
        theta1old = ThetaOld(1);
        theta2old = ThetaOld(2);
        theta3old = ThetaOld(3);
        theta4old = ThetaOld(4);
        theta5old = ThetaOld(5);
        theta6old = ThetaOld(6);
        %
        t1 = linspace(theta1old,theta1,n); 
        t2 = linspace(theta2old,theta2,n); 
        t3 = linspace(theta3old,theta3,n);% -180;  
        t4 = linspace(theta4old,theta4,n); 
        t5 = linspace(theta5old,theta5,n); 
        t6 = linspace(theta6old,theta6,n); 

        n = length(t1);
        for i = 2:1:n
            % Forward Kinematics
            %
            T_01 = tmat(0, 0, 0, t1(i));
            T_12 = tmat(-90, 0, 0, t2(i));
            T_23 = tmat(0, a2, d3, t3(i));
            T_34 = tmat(-90, a3, d4, t4(i));
            T_45 = tmat(90, 0, 0, t5(i));
            T_56 = tmat(-90, 0, 0, t6(i));

% 
%             %     T_67 = [   1            0      0 0
%             %                0            1      0 0
%             %                0            0      1 188
%             %                0            0      0 1];

            %T_01 = T_01;  % it is, but don't need to say so.
            T_02 = T_01*T_12;
            T_03 = T_02*T_23;
            T_04 = T_03*T_34;
            T_05 = T_04*T_45;
            T_06 = T_05*T_56;
            %     T_07 = T_06*T_67;
            %
            s1 = getappdata(0,'Link1_data');
            s2 = getappdata(0,'Link2_data');
            s3 = getappdata(0,'Link3_data');
            s4 = getappdata(0,'Link4_data');
            s5 = getappdata(0,'Link5_data');
            s6 = getappdata(0,'Link6_data');
            s7 = getappdata(0,'Link7_data');
            %A1 = getappdata(0,'Area_data');

            Link1 = s1.V1;
            Link2 = (T_01*s2.V2')';
            Link3 = (T_02*s3.V3')';
            Link4 = (T_03*s4.V4')';
            Link5 = (T_04*s5.V5')';
            Link6 = (T_05*s6.V6')';
            Link7 = (T_06*s7.V7')';
            %     Tool = T_07;

            %     if sqrt(Tool(1,4)^2+Tool(2,4)^2)<514
            %         Err2 = 1;
            %         break
            %     end
            %
            handles = getappdata(0,'patch_h');           %
            L1 = handles(1);
            L2 = handles(2);
            L3 = handles(3);
            L4 = handles(4);
            L5 = handles(5);
            L6 = handles(6);
            L7 = handles(7);
            Tr = handles(9);
            %
            set(L1,'vertices',Link1(:,1:3),'facec', [0.717,0.116,0.123]);
            set(L1, 'EdgeColor','none');
            set(L2,'vertices',Link2(:,1:3),'facec', [0.216,1,.583]);
            set(L2, 'EdgeColor','none');
            set(L3,'vertices',Link3(:,1:3),'facec', [0.306,0.733,1]);
            set(L3, 'EdgeColor','none');
            set(L4,'vertices',Link4(:,1:3),'facec', [1,0.542,0.493]);
            set(L4, 'EdgeColor','none');
            set(L5,'vertices',Link5(:,1:3),'facec', [0.216,1,.583]);
            set(L5, 'EdgeColor','none');
            set(L6,'vertices',Link6(:,1:3),'facec', [1,1,0.255]);
            set(L6, 'EdgeColor','none');
            set(L7,'vertices',Link7(:,1:3),'facec', [0.306,0.733,1]);
            set(L7, 'EdgeColor','none');
            % store trail in appdata 
            if trail == 'y'
                x_trail = getappdata(0,'xtrail');
                y_trail = getappdata(0,'ytrail');
                z_trail = getappdata(0,'ztrail');
                %
                xdata = [x_trail T_04(1,4)];
                ydata = [y_trail T_04(2,4)];
                zdata = [z_trail T_04(3,4)];
                %
                setappdata(0,'xtrail',xdata); % used for trail tracking.
                setappdata(0,'ytrail',ydata); % used for trail tracking.
                setappdata(0,'ztrail',zdata); % used for trail tracking.
                %
                set(Tr,'xdata',xdata,'ydata',ydata,'zdata',zdata);
            end
            drawnow
        end
        setappdata(0,'ThetaOld',[theta1,theta2,theta3,theta4,theta5,theta6]);
    end
%%
%
%
%%
    function InitHome
        % Use forward kinematics to place the robot in a specified
        % configuration.
        % Figure setup data, create a new figure for the GUI
        set(0,'Units','pixels')
        dim = get(0,'ScreenSize');
        fig_1 = figure('doublebuffer','on','Position',[0,35,dim(3)-200,dim(4)-110],...
            'MenuBar','none','Name',' 3D Puma Robot Graphical Demo',...
            'NumberTitle','off','CloseRequestFcn',@del_app);
        hold on;
        %light('Position',[-1 0 0]);
        light                               % add a default light
        daspect([1 1 1])                    % Setting the aspect ratio
        view(135,25)
        xlabel('X'),ylabel('Y'),zlabel('Z');
        title('WWU Robotics Lab PUMA 762');
        axis([-1500 1500 -1500 1500 -1120 1500]);
        plot3([-1500,1500],[-1500,-1500],[-1120,-1120],'k')
        plot3([-1500,-1500],[-1500,1500],[-1120,-1120],'k')
        plot3([-1500,-1500],[-1500,-1500],[-1120,1500],'k')
        plot3([-1500,-1500],[1500,1500],[-1120,1500],'k')
        plot3([-1500,1500],[-1500,-1500],[1500,1500],'k')
        plot3([-1500,-1500],[-1500,1500],[1500,1500],'k')

        s1 = getappdata(0,'Link1_data');
        s2 = getappdata(0,'Link2_data');
        s3 = getappdata(0,'Link3_data');
        s4 = getappdata(0,'Link4_data');
        s5 = getappdata(0,'Link5_data');
        s6 = getappdata(0,'Link6_data');
        s7 = getappdata(0,'Link7_data');
        A1 = getappdata(0,'Area_data');
        %
        a2 = 650;
        a3 = 0;
        d3 = 190;
        d4 = 600;
        Px = 5000;
        Py = 5000;
        Pz = 5000;

        %The 'home' position, for init.
        t1 = 90;
        t2 = -90;
        t3 = -90;
        t4 = 0;
        t5 = 0;
        t6 = 0;
        
        % Forward Kinematics
        T_01 = tmat(0, 0, 0, t1);
        T_12 = tmat(-90, 0, 0, t2);
        T_23 = tmat(0, a2, d3, t3);
        T_34 = tmat(-90, a3, d4, t4);
        T_45 = tmat(90, 0, 0, t5);
        T_56 = tmat(-90, 0, 0, t6);

        % Each link fram to base frame transformation
        T_02 = T_01*T_12;
        T_03 = T_02*T_23;
        T_04 = T_03*T_34;
        T_05 = T_04*T_45;
        T_06 = T_05*T_56;
        
        % Actual vertex data of robot links
        Link1 = s1.V1;
        Link2 = (T_01*s2.V2')';
        Link3 = (T_02*s3.V3')';
        Link4 = (T_03*s4.V4')';
        Link5 = (T_04*s5.V5')';
        Link6 = (T_05*s6.V6')';
        Link7 = (T_06*s7.V7')';
        
        % points are no fun to watch, make it look 3d.
        L1 = patch('faces', s1.F1, 'vertices' ,Link1(:,1:3));
        L2 = patch('faces', s2.F2, 'vertices' ,Link2(:,1:3));
        L3 = patch('faces', s3.F3, 'vertices' ,Link3(:,1:3));
        L4 = patch('faces', s4.F4, 'vertices' ,Link4(:,1:3));
        L5 = patch('faces', s5.F5, 'vertices' ,Link5(:,1:3));
        L6 = patch('faces', s6.F6, 'vertices' ,Link6(:,1:3));
        L7 = patch('faces', s7.F7, 'vertices' ,Link7(:,1:3));
        A1 = patch('faces', A1.Fa, 'vertices' ,A1.Va(:,1:3));
        Tr = plot3(0,0,0,'b.'); % holder for trail paths
        %
        setappdata(0,'patch_h',[L1,L2,L3,L4,L5,L6,L7,A1,Tr])
        %
        setappdata(0,'xtrail',0); % used for trail tracking.
        setappdata(0,'ytrail',0); % used for trail tracking.
        setappdata(0,'ztrail',0); % used for trail tracking.
        %
        set(L1, 'facec', [0.717,0.116,0.123]);
        set(L1, 'EdgeColor','none');
        set(L2, 'facec', [0.216,1,.583]);
        set(L2, 'EdgeColor','none');
        set(L3, 'facec', [0.306,0.733,1]);
        set(L3, 'EdgeColor','none');
        set(L4, 'facec', [1,0.542,0.493]);
        set(L4, 'EdgeColor','none');
        set(L5, 'facec', [0.216,1,.583]);
        set(L5, 'EdgeColor','none');
        set(L6, 'facec', [1,1,0.255]);
        set(L6, 'EdgeColor','none');
        set(L7, 'facec', [0.306,0.733,1]);
        set(L7, 'EdgeColor','none');
        set(A1, 'facec', [.8,.8,.8],'FaceAlpha',.25);
        set(A1, 'EdgeColor','none');
        %
        setappdata(0,'ThetaOld',[90,-90,-90,0,0,0]);
        %
    end
%%
    function T = tmat(alpha, a, d, theta)
        % tmat(alpha, a, d, theta) (T-Matrix used in Robotics)
        % The homogeneous transformation called the "T-MATRIX"
        % as used in the Kinematic Equations for robotic type
        % systems (or equivalent).
        %
        % This is equation 3.6 in Craig's "Introduction to Robotics."
        % alpha, a, d, theta are the Denavit-Hartenberg parameters.
        %
        % (NOTE: ALL ANGLES MUST BE IN DEGREES.)
        %
        alpha = alpha*pi/180;    %Note: alpha is in radians.
        theta = theta*pi/180;    %Note: theta is in radians.
        c = cos(theta);
        s = sin(theta);
        ca = cos(alpha);
        sa = sin(alpha);
        T = [c -s 0 a; s*ca c*ca -sa -sa*d; s*sa c*sa ca ca*d; 0 0 0 1];
    end
%%
    function del_app(varargin)
        %This is the main figure window close function, to remove any
        % app data that may be left due to using it for geometry.
        %CloseRequestFcn
        % here is the data to remove:
        %     Link1_data: [1x1 struct]
        %     Link2_data: [1x1 struct]
        %     Link3_data: [1x1 struct]
        %     Link4_data: [1x1 struct]
        %     Link5_data: [1x1 struct]
        %     Link6_data: [1x1 struct]
        %     Link7_data: [1x1 struct]
        %      Area_data: [1x1 struct]
        %        patch_h: [1x9 double]
        %       ThetaOld: [90 -182 -90 -106 80 106]
        %         xtrail: 0
        %         ytrail: 0
        %         ztrail: 0
        % Now remove them.
        rmappdata(0,'Link1_data');
        rmappdata(0,'Link2_data');
        rmappdata(0,'Link3_data');
        rmappdata(0,'Link4_data');
        rmappdata(0,'Link5_data');
        rmappdata(0,'Link6_data');
        rmappdata(0,'Link7_data');
        rmappdata(0,'ThetaOld');
        rmappdata(0,'Area_data');
        rmappdata(0,'patch_h');
        rmappdata(0,'xtrail');
        rmappdata(0,'ytrail');
        rmappdata(0,'ztrail');
        delete(fig_1);
    end
        
%%
    function [hout,ax_out] = uibutton(varargin)
        %uibutton: Create pushbutton with more flexible labeling than uicontrol.
        % Usage:
        %   uibutton accepts all the same arguments as uicontrol except for the
        %   following property changes:
        %
        %     Property      Values
        %     -----------   ------------------------------------------------------
        %     Style         'pushbutton', 'togglebutton' or 'text', default =
        %                   'pushbutton'.
        %     String        Same as for text() including cell array of strings and
        %                   TeX or LaTeX interpretation.
        %     Interpreter   'tex', 'latex' or 'none', default = default for text()
        %
        % Syntax:
        %   handle = uibutton('PropertyName',PropertyValue,...)
        %   handle = uibutton(parent,'PropertyName',PropertyValue,...)
        %   [text_obj,axes_handle] = uibutton('Style','text',...
        %       'PropertyName',PropertyValue,...)
        %
        % uibutton creates a temporary axes and text object containing the text to
        % be displayed, captures the axes as an image, deletes the axes and then
        % displays the image on the uicontrol.  The handle to the uicontrol is
        % returned.  If you pass in a handle to an existing uicontol as the first
        % argument then uibutton will use that uicontrol and not create a new one.
        %
        % If the Style is set to 'text' then the axes object is not deleted and the
        % text object handle is returned (as well as the handle to the axes in a
        % second output argument).
        %
        % See also UICONTROL.

        % Version: 1.6, 20 April 2006
        % Author:  Douglas M. Schwarz
        % Email:   dmschwarz=ieee*org, dmschwarz=urgrad*rochester*edu
        % Real_email = regexprep(Email,{'=','*'},{'@','.'})


        % Detect if first argument is a uicontrol handle.
        keep_handle = false;
        if nargin > 0
            h = varargin{1};
            if isscalar(h) && ishandle(h) && strcmp(get(h,'Type'),'uicontrol')
                keep_handle = true;
                varargin(1) = [];
            end
        end

        % Parse arguments looking for 'Interpreter' property.  If found, note its
        % value and then remove it from where it was found.
        interp_value = get(0,'DefaultTextInterpreter');
        arg = 1;
        remove = [];
        while arg <= length(varargin)
            v = varargin{arg};
            if isstruct(v)
                fn = fieldnames(v);
                for i = 1:length(fn)
                    if strncmpi(fn{i},'interpreter',length(fn{i}))
                        interp_value = v.(fn{i});
                        v = rmfield(v,fn{i});
                    end
                end
                varargin{arg} = v;
                arg = arg + 1;
            elseif ischar(v)
                if strncmpi(v,'interpreter',length(v))
                    interp_value = varargin{arg+1};
                    remove = [remove,arg,arg+1];
                end
                arg = arg + 2;
            elseif arg == 1 && isscalar(v) && ishandle(v) && ...
                    any(strcmp(get(h,'Type'),{'figure','uipanel'}))
                arg = arg + 1;
            else
                error('Invalid property or uicontrol parent.')
            end
        end
        varargin(remove) = [];

        % Create uicontrol, get its properties then hide it.
        if keep_handle
            set(h,varargin{:})
        else
            h = uicontrol(varargin{:});
        end
        s = get(h);
        if ~any(strcmp(s.Style,{'pushbutton','togglebutton','text'}))
            delete(h)
            error('''Style'' must be pushbutton, togglebutton or text.')
        end
        set(h,'Visible','off')

        % Create axes.
        parent = get(h,'Parent');
        ax = axes('Parent',parent,...
            'Units',s.Units,...
            'Position',s.Position,...
            'XTick',[],'YTick',[],...
            'XColor',s.BackgroundColor,...
            'YColor',s.BackgroundColor,...
            'Box','on',...
            'Color',s.BackgroundColor);
        % Adjust size of axes for best appearance.
        set(ax,'Units','pixels')
        pos = round(get(ax,'Position'));
        if strcmp(s.Style,'text')
            set(ax,'Position',pos + [0 1 -1 -1])
        else
            set(ax,'Position',pos + [4 4 -8 -8])
        end
        switch s.HorizontalAlignment
            case 'left'
                x = 0.0;
            case 'center'
                x = 0.5;
            case 'right'
                x = 1;
        end
        % Create text object.
        text_obj = text('Parent',ax,...
            'Position',[x,0.5],...
            'String',s.String,...
            'Interpreter',interp_value,...
            'HorizontalAlignment',s.HorizontalAlignment,...
            'VerticalAlignment','middle',...
            'FontName',s.FontName,...
            'FontSize',s.FontSize,...
            'FontAngle',s.FontAngle,...
            'FontWeight',s.FontWeight,...
            'Color',s.ForegroundColor);

        % If we are creating something that looks like a text uicontrol then we're
        % all done and we return the text object and axes handles rather than a
        % uicontrol handle.
        if strcmp(s.Style,'text')
            delete(h)
            if nargout
                hout = text_obj;
                ax_out = ax;
            end
            return
        end

        % Capture image of axes and then delete the axes.
        frame = getframe(ax);
        delete(ax)

        % Build RGB image, set background pixels to NaN and put it in 'CData' for
        % the uicontrol.
        if isempty(frame.colormap)
            rgb = frame.cdata;
        else
            rgb = reshape(frame.colormap(frame.cdata,:),[pos([4,3]),3]);
        end
        size_rgb = size(rgb);
        rgb = double(rgb)/255;
        back = repmat(permute(s.BackgroundColor,[1 3 2]),size_rgb(1:2));
        isback = all(rgb == back,3);
        rgb(repmat(isback,[1 1 3])) = NaN;
        set(h,'CData',rgb,'String','','Visible',s.Visible)

        % Assign output argument if necessary.
        if nargout
            hout = h;
        end
%%
    end
end
% Finally.