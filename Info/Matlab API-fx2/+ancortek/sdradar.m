classdef sdradar < handle
    %sdradar Matlab API for Ancortek SDR-KITs
    %
    % Dependencies: 
    %     Matlab: R2017b or later
    %
    % Supported OS:
    %    macOS, Windows
    %     
    %   Usage: refer to 'UsageDemo.m'
        
    properties    
        model_s % full model number in string
        rf_sn % RF board serial number
        pm_sn % PM board serial number
        nTx_max % maximum number of transmitter(s)
        nRx_max % maximum number of receiver(s)  
        nTx % number of active transmitter(s)
        nRx % number of active receiver(s)
        sTime % sweep time in second
        Tramp_up % ramp-up time in second in FMCW sawtooth/triangle
        Tramp_dn % ramp-down time in second in FMCW triangle
        nSamp % samples per sweep per channel
        nSamp_up % number of samples in the up-chirp part of the FMCW sawtooth
        nSamp_max % maximum samples per sweep per channel
        fStart % in Hz
        fStop % in Hz
        fMin % lower hardware frequency limit in Hz
        fMax % upper hardware frequency limit in Hz
        bw_max_GHz % maximum sweeping bandwidth in GHz. Its value depends on the sweep time ('sTime')        
        modulation_pre % Previous modulation scheme
    end
    
    properties (SetObservable)%, AbortSet)
% Setting this property sets up the radar's modulation scheme. It accepts an ancortek.modOpt enum object, such as ancortek.modOpt.FMCW_sawtooth
% Available selections:
% ancortek.modOpt.FMCW_sawtooth
% ancortek.modOpt.FMCW_triangle
% ancortek.modOpt.Cw        
        modulation 

% Setting this property sets up the radar's sweep time. It accepts an ancortek.stOpt enum object, such as ancortek.stOpt.us1000
% Available selections:
% ancortek.stOpt.us125: 125 microseconds
% ancortek.stOpt.us250: 250 microseconds        
% ancortek.stOpt.us500: 500 microseconds                
% ancortek.stOpt.us1000: 1 millisecond
% ancortek.stOpt.us2000: 2 milliseconds
% ancortek.stOpt.us4000: 4 milliseconds
% ancortek.stOpt.us8000: 8 milliseconds        
        sweeptime 
        
% Setting this property sets up the radar's sampling rate, represented as sampling number per sweep. It accepts an ancortek.sampOpt enum object, such as ancortek.sampOpt.oneEighth
% Availabe selections (the actual value depends on the number of active Rx and sweep time):
% ancortek.sampOpt.one: maximum sampling rate
% ancortek.sampOpt.half: half of the maximum sampling rate
% ancortek.sampOpt.oneFourth: a fourth of the maximum sampling rate
% ancortek.sampOpt.oneEighth: one eighth of the maximum sampling rate
        samplesPerSweep 

% Setting this property activates selected transmitter(s). It accepts an ancortek.txOpt enum object, such as ancortek.txOpt.tx1
% Availabe selections (some options are not valide depending on the hardware):
% ancortek.txOpt.tx1: activates transmitter number 1
% ancortek.txOpt.tx2: activates transmitter number 2 
% ancortek.txOpt.tx12: activates transmitter number 1 and 2 in a time-division multiplexing fassion
        activeTx 

% Setting this property activates selected receiver(s). It accepts an ancortek.rxOpt enum object, such as ancortek.rxOpt.rx1
% Availabe selections (some options are not valide depending on the hardware):
% ancortek.rxOpt.rx1: activates receiver number 1
% ancortek.rxOpt.rx2: activates receiver number 2 
% ancortek.rxOpt.rx3: activates receiver number 3
% ancortek.rxOpt.rx4: activates receiver number 4 
% ancortek.rxOpt.rx12: activates receiver number 1 and 2 
% ancortek.rxOpt.rx34: activates receiver number 3 and 4 
% ancortek.rxOpt.rx1234: activates receiver number 1, 2, 3 and 4         
        activeRx
    end

    properties (Dependent)
       fc % carrier frequency in Hz
       lambda % qavelength of the carrier frequency in meter
       bw % bandwidth of up-chirps in FMCW sawtooth. If 0, the modulation becomes CW.
       slope % chirp slope in Hz/second
       fs % sampling rate (number of I and Q samples per second)
       rngMax % maximum unambigous range
       rngRes % range resolution
       vMax % maximum unambigous velocity
       vMax_cw % maximum unambigous velocity for CW
       prf % pulse repition frequency
    end
    
    % dependent property getters
    methods
        function value = get.fc(obj)
            value = (obj.fStart + obj.fStop)/2;
        end
        
        function value = get.lambda(obj)
           value = 3e8/obj.fc; 
        end
        
        function value = get.bw(obj)
            value = obj.fStop - obj.fStart;
        end
        
        function value = get.slope(obj)
            value = obj.bw/obj.Tramp_up;
        end
        
        function value = get.fs(obj)
            value = obj.nSamp/obj.sTime;
        end
        
        function value =  get.rngMax(obj)
            value = 3e8/(2*obj.bw)*obj.nSamp_up/2;
        end
        
        function value = get.rngRes(obj)
            value = 3e8/(2*obj.bw);
        end
        
        function value = get.vMax(obj)
            value = obj.prf/2*3e8/obj.fc/2;
        end
        
        function value = get.vMax_cw(obj)
            value = obj.fs*3e8/obj.fc/2;
        end
        
        function value = get.prf(obj)
            value = 1/obj.sTime/obj.nTx;
        end
    end
    
    methods 
        function obj = sdradar()
        %SDRADAR The class constructor initializes some prpoerty listeners. 
            
            % Define property listeners
            addlistener(obj, 'modulation', 'PostSet', @obj.setMod);
            addlistener(obj, 'sweeptime', 'PostSet', @obj.setSt);
            addlistener(obj, 'samplesPerSweep', 'PostSet', @obj.setSamp);
            addlistener(obj, 'activeTx', 'PostSet', @obj.setTx);
            addlistener(obj, 'activeRx', 'PostSet', @obj.setRx);
        end
        
        function init(obj)
        %INIT This method tries to establish a USB connection to the SDR-KIT and obtain the model informtaion
        % If a model is detected successfully, the default parameters will also be configured.            
            
             try
                [~,~,endian] = computer;
                ancortek.sdradar_init;
                obj.cmd_getInfo = uint16(zeros(obj.cmd_len, 1));
                obj.cmd_getInfo(1:obj.cmd_rep) = obj.getInfo_h;
                ancortek.sdradar_send_data(obj.cmd_getInfo);
                kit_info = ancortek.sdradar_get_data(obj.USB_BUFFER_LEFTOVER+obj.USB_BUFFER_BLOCK_SIZE);
                header_ind = find(kit_info == uint16(hex2dec('FA07')));
                if isempty(header_ind)
                    % RF board serial number extraction
                    header_ind = find(kit_info == uint16(hex2dec('FA0D')), 1);
                    if isempty(header_ind)
                       error('Serial number not found...') 
                    end
                    model_c = num2str(kit_info(header_ind+8),'%02u');
                    tmp = dec2hex((kit_info(header_ind+9)));
                    nTx_c = tmp(1);
                    obj.nTx_max = str2double(nTx_c);
                    nRx_c = tmp(2);
                    obj.nRx_max = str2double(nRx_c);
                    series_c = native2unicode(hex2dec(tmp(3:4)), 'UTF-8');
                    
                    if endian == 'L'
                        tmp = typecast(kit_info(header_ind+10),'uint8');
                        year = num2str(tmp(1), '%02u');
                        month = num2str(tmp(2), '%02u');
                        
                        tmp = typecast(kit_info(header_ind+11),'uint8');
                        brdIndLow = num2str(tmp(1), '%02u');
                        brdIndHi = num2str(tmp(2), '%02u');
                    else
                        tmp = typecast(kit_info(header_ind+10),'uint8');
                        month = num2str(tmp(1), '%02u');
                        year = num2str(tmp(2), '%02u');
                        
                        tmp = typecast(kit_info(header_ind+11),'uint8');
                        brdIndLow = num2str(tmp(2), '%02u');
                        brdIndHi = num2str(tmp(1), '%02u');
                    end
                    
                    obj.rf_sn = strcat('RF', sprintf('%04s',model_c), nTx_c, nRx_c, series_c,...
                        month, year, brdIndHi, brdIndLow);
                    
                    if series_c == 'A'
                        if obj.nRx_max > 1
                            if nTx_c == '2'
                                obj.model_s = 'SDR-KIT 2400T2R4';
                            else
                                obj.model_s = ['SDR-KIT ', model_c,'AD',nRx_c];
                            end
                        else
                            obj.model_s = ['SDR-KIT ', model_c,'AD'];
                        end
                    elseif series_c == 'B'
                        obj.model_s = ['SDR-KIT ', model_c,'B'];
                    end
                    fprintf('RF S/N: %s \n' ,obj.rf_sn);                  
                    
                    % PM board serial number extraction
                    tmp = dec2hex(kit_info(header_ind+4),4);
                    month = hex2dec(tmp(1:2));
                    year = hex2dec(tmp(3:4));
                    tmp = dec2hex(kit_info(header_ind+5),4);
                    brdIndHi = hex2dec(tmp(1:2));
                    brdIndLow = hex2dec(tmp(3:4));
                    obj.pm_sn = strcat('PM',dec2hex(kit_info(header_ind+3),4), ...
                        num2str(month,'%02u'),num2str(year,'%02u'),num2str(brdIndHi,'%02u'),num2str(brdIndLow,'%02u'));
                    fprintf('PM S/N: %s \n' ,obj.pm_sn);  
                    
                else
                    % PM board serial number extraction
                    tmp = dec2hex(kit_info(header_ind+4),4);
                    month = hex2dec(tmp(1:2));
                    year = hex2dec(tmp(3:4));
                    tmp = dec2hex(kit_info(header_ind+5),4);
                    brdIndHi = hex2dec(tmp(1:2));
                    brdIndLow = hex2dec(tmp(3:4));
                    obj.pm_sn = strcat('PM',dec2hex(kit_info(header_ind+3),4), ...
                        num2str(month,'%02u'),num2str(year,'%02u'),num2str(brdIndHi,'%02u'),num2str(brdIndLow,'%02u'));
                    fprintf('PM S/N: %s \n' ,obj.pm_sn);                                        
                end                
                       
                if isempty(obj.model_s)
                   error('No RF board detected...')                    
                end
                    
                switch obj.model_s
                    case 'SDR-KIT 240AD'
                        % Hardware limits
                        obj.fMin = 2.1e9;
                        obj.fMax = 2.6e9;
                        obj.bw_max_GHz = 0.5;
                        obj.nTx_max = 1;
                        obj.nRx_max = 1;

                        % Property init
                        obj.sTime = 1e-3;
                        obj.nRx = 1;
                        
                        % Default radar parameters
                        obj.modulation = ancortek.modOpt.FMCW_sawtooth;
                        obj.activeRx = ancortek.rxOpt.rx1;
                        obj.activeTx = ancortek.txOpt.tx1;
                        obj.sweeptime = ancortek.stOpt.us1000;
                        obj.samplesPerSweep = ancortek.sampOpt.oneEighth;

                    case 'SDR-KIT 580AD'
                        % Hardware limits
                        obj.fMin = 5.2e9;
                        obj.fMax = 6.0e9;                       
                        obj.bw_max_GHz = 0.8;
                        obj.nTx_max = 1;
                        obj.nRx_max = 1;
                        
                        % Default radar parameters 
                        obj.modulation = ancortek.modOpt.FMCW_sawtooth;
                        obj.activeRx = ancortek.rxOpt.rx1;
                        obj.activeTx = ancortek.txOpt.tx1;
                        obj.sweeptime = ancortek.stOpt.us1000;
                        obj.samplesPerSweep = ancortek.sampOpt.oneEighth;
                        
                    case 'SDR-KIT 580AD2'
                        % Hardware limits
                        obj.fMin = 5.2e9;
                        obj.fMax = 6.0e9;                  
                        obj.bw_max_GHz = 0.8;
                        obj.nTx_max = 1;
                        obj.nRx_max = 2;
                        
                        % Default radar parameters
                        obj.modulation = ancortek.modOpt.FMCW_sawtooth;
                        obj.activeRx = ancortek.rxOpt.rx12;
                        obj.activeTx = ancortek.txOpt.tx1;
                        obj.sweeptime = ancortek.stOpt.us1000;
                        obj.samplesPerSweep = ancortek.sampOpt.oneEighth;

                    case 'SDR-KIT 980AD'
                        % Hardware limits
                        obj.fMin = 9e9;
                        obj.fMax = 10e9;                       
                        obj.bw_max_GHz = 1;
                        obj.nTx_max = 1;
                        obj.nRx_max = 1;
                        
                        % Default radar parameters
                        obj.modulation = ancortek.modOpt.FMCW_sawtooth;
                        obj.activeRx = ancortek.rxOpt.rx1;
                        obj.activeTx = ancortek.txOpt.tx1;                         
                        obj.sweeptime = ancortek.stOpt.us1000;
                        obj.samplesPerSweep = ancortek.sampOpt.oneEighth;

                  case 'SDR-KIT 980AD2'
                        % Hardware limits
                        obj.fMin = 9e9;
                        obj.fMax = 10e9;                       
                        obj.bw_max_GHz = 1;
                        obj.nTx_max = 1;
                        obj.nRx_max = 2;
                        
                        % Default radar parameters
                        obj.modulation = ancortek.modOpt.FMCW_sawtooth;
                        obj.activeRx = ancortek.rxOpt.rx12;
                        obj.activeTx = ancortek.txOpt.tx1;
                        obj.sweeptime = ancortek.stOpt.us1000;
                        obj.samplesPerSweep = ancortek.sampOpt.oneEighth;
                        
                    case 'SDR-KIT 2400AD'
                        % Hardware limits
                        obj.fMin = 24e9;
                        obj.fMax = 26e9;                       
                        obj.bw_max_GHz = 2;
                        obj.nTx_max = 1;
                        obj.nRx_max = 1;
                                               
                        % Default radar parameters
                        obj.modulation = ancortek.modOpt.FMCW_sawtooth;
                        obj.activeRx = ancortek.rxOpt.rx1;
                        obj.activeTx = ancortek.txOpt.tx1;                                     
                        obj.sweeptime = ancortek.stOpt.us1000;
                        obj.samplesPerSweep = ancortek.sampOpt.oneEighth;
                                                
                    case 'SDR-KIT 2400AD2'
                        % Hardware limits
                        obj.fMin = 24e9;
                        obj.fMax = 26e9;
                        obj.bw_max_GHz = 2;
                        obj.nTx_max = 1;
                        obj.nRx_max = 2;
                                               
                        % Default radar parameters
                        obj.modulation = ancortek.modOpt.FMCW_sawtooth;
                        obj.activeRx = ancortek.rxOpt.rx12;
                        obj.activeTx = ancortek.txOpt.tx1;
                        obj.sweeptime = ancortek.stOpt.us1000;
                        obj.samplesPerSweep = ancortek.sampOpt.oneEighth;
                        
                    case 'SDR-KIT 2400AD4'
                        % Hardware limits
                        obj.fMin = 24e9;
                        obj.fMax = 26e9;                      
                        obj.bw_max_GHz = 2;
                        obj.nTx_max = 1;
                        obj.nRx_max = 4;
                                               
                        % Default radar parameters
                        obj.modulation = ancortek.modOpt.FMCW_sawtooth;
                        obj.activeRx = ancortek.rxOpt.rx1234;
                        obj.activeTx = ancortek.txOpt.tx1;
                        obj.sweeptime = ancortek.stOpt.us1000;
                        obj.samplesPerSweep = ancortek.sampOpt.oneEighth;
                        
                    case 'SDR-KIT 2400T2R4'
                        % Hardware limits
                        obj.fMin = 24e9;
                        obj.fMax = 26e9;
                        obj.bw_max_GHz = 2;
                        obj.nTx_max = 2;
                        obj.nRx_max = 4;
                                               
                        % Default radar parameters 
                        obj.modulation = ancortek.modOpt.FMCW_sawtooth;
                        obj.activeRx = ancortek.rxOpt.rx1234;
                        obj.activeTx = ancortek.txOpt.tx12;
                        obj.sweeptime = ancortek.stOpt.us1000;
                        obj.samplesPerSweep = ancortek.sampOpt.oneEighth;
                    otherwise
                       error('Model not recognized')
                end
                
                fprintf('Initialization succeeded.\n')
                
             catch ME
                 error(ME.message);
             end
         end


        function bw_update(obj, fStart, fStop)
        %BW_UPDATE This method sets up the FMCW bandwidth. It works with 
        %FMCW sawtooth, FMCW triangle and CW

            if fStart < obj.fMin || fStop > obj.fMax
               error('Frequency is out of limit. \n Min. Freq.: %.2f GHz \n Max. Freq.: %.2f GHz \n', ...
                   obj.fMin/1e9, obj.fMax/1e9);
            end
            
            if (fStop - fStart)/1e9 > obj.bw_max_GHz
                error('Bandwidth is out of allowable range given the current sweep time. \n Given the current sweep time is %.3f ms, the maximum bandwidth is %u MHz', obj.sTime*1e3, obj.bw_max_GHz*1e3);
            end
            
            obj.fStart = fStart;
            obj.fStop = fStop;
            
            if fStart == fStop
                obj.modulation = ancortek.modOpt.CW;
                
                if obj.nTx == 2
                    % Prevent multiple transmitter from operating in CW mode
                    obj.activeTx = ancortek.txOpt.tx1;
                end
                
                obj.PLL_register_setup_sawtooth;
                
            elseif obj.modulation_pre == ancortek.modOpt.FMCW_sawtooth
                obj.modulation = ancortek.modOpt.FMCW_sawtooth;
                obj.PLL_register_setup_sawtooth;
                
            elseif obj.modulation_pre == ancortek.modOpt.FMCW_triangle
                obj.modulation = ancortek.modOpt.FMCW_triangle;
                obj.PLL_register_setup_triangle;
                
            end
            
            fprintf('Bandwidth was updated successfully. \nfStart: %.2f GHz; fStop: %.2f GHz.\n', fStart/1e9, fStop/1e9);
        end
        
        
        
         function [Is, Qs] = get_IQCube(obj, PN)
            %get_IQCube Get ADC samples from I and Q channels
            %
            %    input:
            %    PN: an integer value for the number of pulses (sweeps)
            %
            %    return:
            %    Is: a radar data cube from I channel. Its dimension is NumberOfSamplersPerSweep by NumberOfReceivers by NumberOfSweeps (FastSamples by SpatialSamples by SlowSamples)
            %    Qs: a radar data cube from Q channel. Its dimension is NumberOfSamplersPerSweep by NumberOfReceivers by NumberOfSweeps (FastSamples by SpatialSamples by SlowSamples)
            %
            % Note:
            %    If the number of active transmitter (obj.nTx) is 2, the radar operates in a time-multiplexing fashion. Spatial samples corresponding to Tx1 and Tx2 are alternated in the SpatialSamples dimension
            
            % need to send something to the kit before each data
            % requesting session
            ancortek.sdradar_send_data(uint16(zeros(obj.cmd_len, 1)));
                
            % * 2: I and Q two channels
            % + 2048: leftover samples in the USB buffer
            % Take extra sweeps to search for the first header and make sure it's a
            % multiple of 512
            data_length = ceil((PN + 10) * obj.nSamp * 2 * obj.nRx * obj.nTx/ 512)*512 + 2048;
            rawdata = ancortek.sdradar_get_data(data_length);
            
            % Discard 2048 leftover data samples
            rawdata = double(rawdata(2049:end));
            
            % Data check and remove headers
            ind_Tx1 = find(rawdata >= 49152);
            ind_Tx1_diff = diff(ind_Tx1);
            
            if any(ind_Tx1_diff ~= obj.nSamp*obj.nRx*obj.nTx*2) || isempty(ind_Tx1)
                fprintf('Data loss occurred in Tx 1. \n')
                return
            end
            
            % remove headers
            rawdata(ind_Tx1) = rawdata(ind_Tx1) - 49152;
            
            if obj.nTx == 2
                ind_Tx2 = find(rawdata < 49152 & rawdata >= 32768);
                ind_Tx2_diff = diff(ind_Tx2);
                
                if any(ind_Tx2_diff ~= obj.nSamp*obj.nRx*obj.nTx*2) || isempty(ind_Tx2) 
                    fprintf('Data loss occurred in Tx 2. \n');                        
                    return
                end
                
                % remove headers
                rawdata(ind_Tx2) = rawdata(ind_Tx2) - 32768;      
            end
               
            % readjust the first sample to the beginning of the first sweep and format the data into complex values            
            raw = rawdata(ind_Tx1(1):ind_Tx1(1)+obj.nSamp*obj.nRx*obj.nTx*PN*2-1);
            
            % This doesn't work becuase ind_Tx2(1) > ind_Tx1(1) is possible.
            % raw = rawdata(ind_Tx1(1):ind_Tx2(PN)+obj.nSamp*2*obj.nRx-1);             
            
            Is = raw(2:2:end);
            Qs = raw(1:2:end);
            Is = reshape(Is, obj.nRx, obj.nSamp, PN*obj.nTx);
            Qs = reshape(Qs, obj.nRx, obj.nSamp, PN*obj.nTx);
 
            Is = permute(Is, [2 1 3]);
            Qs = permute(Qs, [2 1 3]);

            if obj.modulation == ancortek.modOpt.FMCW_sawtooth
                  % remove down-chirp samples
                  Is = Is(1:obj.nSamp_up, :, :);
                  Qs = Qs(1:obj.nSamp_up, :, :);                
                  
                  % remove DC offset
                  Is = bsxfun(@minus, Is, mean(Is, 1));
                  Qs = bsxfun(@minus, Qs, mean(Qs, 1));
                  
            elseif obj.modulation == ancortek.modOpt.FMCW_triangle
                % remove DC offset
                  Is = bsxfun(@minus, Is, mean(Is, 1));
                  Qs = bsxfun(@minus, Qs, mean(Qs, 1));
            end

            % Rx3 from 2400T2R4 needs to be flipped
%             if obj.activeRx == ancortek.rxOpt.rx34
%                 Is(:,1,:) = -Is(:,1,:);
%                 Qs(:,1,:) = -Qs(:,1,:);                
%             end
%                 
%             if obj.activeRx == ancortek.rxOpt.rx1234
%                 Is(:,3,:) = -Is(:,3,:);
%                 Qs(:,3,:) = -Qs(:,3,:);
%             end            
        end
    end    

    
    properties (Constant, Hidden=true)
        USB_BUFFER_BLOCK_SIZE = 512;  % in bytes
        USB_BUFFER_LEFTOVER = 1024; % in bytes
        max_data_rate = 2048000; % uint_16 samples per second
    end

    properties (Access = private)
        
        % Info request command header
        getInfo_h = uint16(hex2dec('FA00'));
        
        % General setting command headers
        setMod_h = uint16(hex2dec('E100'));
        setSt_h = uint16(hex2dec('E200'));
        setSamp_h = uint16(hex2dec('E300'));
        setTx_h = uint16(hex2dec('E400'));
        setRx_h = uint16(hex2dec('E500'));
        
        % PLL setting command headers
        setSweepstop_Hi_h = uint16(hex2dec('D100'));
        setSweepstop_Lo_h = uint16(hex2dec('D200'));
        setReg03_Hi_h = uint16(hex2dec('C100'));
        setReg03_Mi_h = uint16(hex2dec('C200'));
        setReg03_Lo_h = uint16(hex2dec('C300'));
        setReg04_Hi_h = uint16(hex2dec('C400'));
        setReg04_Mi_h = uint16(hex2dec('C500'));
        setReg04_Lo_h = uint16(hex2dec('C600'));
        setReg0A_Hi_h = uint16(hex2dec('C700'));
        setReg0A_Mi_h = uint16(hex2dec('C800'));
        setReg0A_Lo_h = uint16(hex2dec('C900'));
        setReg0C_Hi_h = uint16(hex2dec('CA00'));
        setReg0C_Mi_h = uint16(hex2dec('CB00'));
        setReg0C_Lo_h = uint16(hex2dec('CC00'));
        setReg0D_Hi_h = uint16(hex2dec('CD00'));
        setReg0D_Mi_h = uint16(hex2dec('CE00'));
        setReg0D_Lo_h = uint16(hex2dec('CF00'));
        
        setSweepstop_Hi
        setSweepstop_Lo
        setReg03_Hi
        setReg03_Mi
        setReg03_Lo
        setReg04_Hi
        setReg04_Mi
        setReg04_Lo
        setReg0A_Hi
        setReg0A_Mi
        setReg0A_Lo
        setReg0C_Hi
        setReg0C_Mi
        setReg0C_Lo
        setReg0D_Hi
        setReg0D_Mi
        setReg0D_Lo
        
        cmd_len = 1024; % Command length in uint16
        cmd_rep =1024 % Command repetition times
        cmd_getInfo % Command for getting device info
        cmd_setMod % Command for setting modulation scheme
        cmd_setSt % Command for setting sweep time
        cmd_setSamp % Command for setting 'samples/sweep'
        cmd_setTx
        cmd_setRx
        % cmd_setPll % for combining all pll registers
        cmd_setSweepstop_Hi
        cmd_setSweepstop_Lo
        cmd_setReg03_Hi
        cmd_setReg03_Mi
        cmd_setReg03_Lo
        cmd_setReg04_Hi
        cmd_setReg04_Mi
        cmd_setReg04_Lo
        cmd_setReg0A_Hi
        cmd_setReg0A_Mi
        cmd_setReg0A_Lo
        cmd_setReg0C_Hi
        cmd_setReg0C_Mi
        cmd_setReg0C_Lo
        cmd_setReg0D_Hi
        cmd_setReg0D_Mi
        cmd_setReg0D_Lo
    end


    methods (Access = private)
        function setMod(obj,~,~)
        %setMod Set the modulation scheme
        
            % record previous modulation scheme
            if obj.modulation ~= ancortek.modOpt.CW
                obj.modulation_pre = obj.modulation;
            end
            
            obj.cmd_setMod = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setMod(1:obj.cmd_rep) = obj.setMod_h + obj.modulation;
            ancortek.sdradar_send_data(obj.cmd_setMod);
            
            switch obj.modulation
              case ancortek.modOpt.FMCW_sawtooth   
                fprintf('Modulation scheme is set to FMCW sawtooth.\n')
              case ancortek.modOpt.FMCW_triangle
                fprintf('Modulation scheme is set to FMCW triangle.\n')
              case ancortek.modOpt.FSK
                fprintf('Modulation scheme is set to FSK.\n')
              case ancortek.modOpt.CW
                fprintf('Modulation scheme is set to CW.\n')
            end
        end
        
        function setSt(obj,~,~)
        %setSt Set the sweep time
        %Setting this parameter will also update the 'bw_max_GHz' property and  set the bandwidth to its maximum given the sweep duration.     
            
            switch obj.sweeptime
              case ancortek.stOpt.us125
                obj.sTime = 125e-6;
                fprintf('Sweep time is set to 125 us.\n')
                
                if contains(obj.model_s, '2400')
                    obj.bw_max_GHz = 0.25;
                    obj.bw_update(24e9, 24.25e9)
                elseif contains(obj.model_s, '240AD')
                    obj.bw_max_GHz = 0.1;
                    obj.bw_update(2.4e9, 2.5e9)
                elseif contains(obj.model_s, '580')
                    obj.bw_max_GHz = 0.1;
                    obj.bw_update(5.6e9, 5.7e9)
                elseif contains(obj.model_s, '980')
                    obj.bw_max_GHz = 0.1;
                    obj.bw_update(9.6e9, 9.7e9)
                end

              case ancortek.stOpt.us250
                obj.sTime = 250e-6;
                fprintf('Sweep time is set to 250 us.\n')

                if contains(obj.model_s, '2400')
                    obj.bw_max_GHz = 0.25;
                    obj.bw_update(24e9, 24.25e9)
                elseif contains(obj.model_s, '240AD')
                    obj.bw_max_GHz = 0.2;
                    obj.bw_update(2.3e9, 2.5e9)
                elseif contains(obj.model_s, '580')
                    obj.bw_max_GHz = 0.2;
                    obj.bw_update(5.6e9, 5.8e9)
                elseif contains(obj.model_s, '980')
                    obj.bw_max_GHz = 0.2;
                    obj.bw_update(9.6e9, 9.8e9)
                end

              case ancortek.stOpt.us500
                obj.sTime = 500e-6;
                fprintf('Sweep time is set to 500 us.\n')

                if contains(obj.model_s, '2400')
                    obj.bw_max_GHz = 1;
                    obj.bw_update(24e9, 25e9)
                elseif contains(obj.model_s, '240AD')
                    obj.bw_max_GHz = 0.4;
                    obj.bw_update(2e9, 2.4e9)
                elseif contains(obj.model_s, '580')
                    obj.bw_max_GHz = 0.4;
                    obj.bw_update(5.6e9, 6.0e9)
                elseif contains(obj.model_s, '980')
                    obj.bw_max_GHz = 0.4;
                    obj.bw_update(9.6e9, 10.0e9)
                end
                
              case ancortek.stOpt.us1000
                obj.sTime = 1000e-6;
                fprintf('Sweep time is set to 1 ms.\n')

                if contains(obj.model_s, '2400')
                    obj.bw_max_GHz = 2;
                    obj.bw_update(24e9, 26e9)
                elseif contains(obj.model_s, '240AD')
                    obj.bw_max_GHz = 0.4;
                    obj.bw_update(2.1e9, 2.5e9)
                elseif contains(obj.model_s, '580')
                    obj.bw_max_GHz = 0.4;
                    obj.bw_update(5.6e9, 6.0e9)
                elseif contains(obj.model_s, '980')
                    obj.bw_max_GHz = 0.4;
                    obj.bw_update(9.6e9, 10.0e9)
                end
                
              case ancortek.stOpt.us2000
                obj.sTime = 2000e-6;
                fprintf('Sweep time is set to 2 ms.\n')

                if contains(obj.model_s, '2400')
                    obj.bw_max_GHz = 2;
                    obj.bw_update(24e9, 26e9)
                elseif contains(obj.model_s, '240AD')
                    obj.bw_max_GHz = 0.5;
                    obj.bw_update(2.1e9, 2.6e9)
                elseif contains(obj.model_s, '580')
                    obj.bw_max_GHz = 0.5;
                    obj.bw_update(5.5e9, 6.0e9)
                elseif contains(obj.model_s, '980')
                    obj.bw_max_GHz = 0.5;
                    obj.bw_update(9.5e9, 10.0e9)
                end

              case ancortek.stOpt.us4000
                obj.sTime = 4000e-6;
                fprintf('Sweep time is set to 4 ms.\n')

                if contains(obj.model_s, '2400')
                    obj.bw_max_GHz = 2;
                    obj.bw_update(24e9, 26e9)
                elseif contains(obj.model_s, '240AD')
                    obj.bw_max_GHz = 0.5;
                    obj.bw_update(2.1e9, 2.6e9)
                elseif contains(obj.model_s, '580')
                    obj.bw_max_GHz = 0.6;
                    obj.bw_update(5.4e9, 6.0e9)
                elseif contains(obj.model_s, '980')
                    obj.bw_max_GHz = 0.6;
                    obj.bw_update(9.4e9, 10.0e9)
                end

              case ancortek.stOpt.us8000
                obj.sTime = 8000e-6;
                fprintf('Sweep time is set to 8 ms.\n')

                if contains(obj.model_s, '2400')
                    obj.bw_max_GHz = 2;
                    obj.bw_update(24e9, 26e9)
                elseif contains(obj.model_s, '240AD')
                    obj.bw_max_GHz = 0.5;
                    obj.bw_update(2.1e9, 2.6e9)
                elseif contains(obj.model_s, '580')
                    obj.bw_max_GHz = 0.8;
                    obj.bw_update(5.2e9, 6.0e9)
                elseif contains(obj.model_s, '980')
                    obj.bw_max_GHz = 1;
                    obj.bw_update(9.0e9, 10.0e9)
                end
            end
            obj.cmd_setSt = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setSt(1:obj.cmd_rep) = obj.setSt_h + obj.sweeptime;
            ancortek.sdradar_send_data(obj.cmd_setSt);
            
        % Update the number of samples per sweep becuase it depends on the number of receiver and sweep time.
            if ~isempty(obj.samplesPerSweep)
                obj.nSamp_max = obj.max_data_rate*obj.sTime/obj.nRx;
                obj.nSamp = obj.nSamp_max * 2^(-double(obj.samplesPerSweep));
                obj.nSamp_up = floor(obj.nSamp * obj.Tramp_up / obj.sTime);
                fprintf('Sampling at %u I and Q samples per sweep, 2 bytes per sample. \n', obj.nSamp)
                fprintf('Sampling rate: %u I and Q samples per second. \n', obj.fs)
            end
        end
        
        function setSamp(obj,~,~)
        %setSamp Set the sampling number per sweep
            
            obj.nSamp_max = obj.max_data_rate*obj.sTime/obj.nRx;
            obj.cmd_setSamp = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setSamp(1:obj.cmd_rep) = obj.setSamp_h + obj.samplesPerSweep;
            ancortek.sdradar_send_data(obj.cmd_setSamp);
            
            switch obj.samplesPerSweep
              case ancortek.sampOpt.oneEighth
                obj.nSamp = obj.nSamp_max * 1/8;
                fprintf('Sampling at %u I and Q samples per sweep, 2 bytes per sample. \n', obj.nSamp)
                fprintf('Sampling rate: %u I and Q samples per second. \n', obj.fs)
              case ancortek.sampOpt.oneFourth
                obj.nSamp = obj.nSamp_max * 1/4;
                fprintf('Sampling at %u I and Q samples per sweep, 2 bytes per sample. \n', obj.nSamp)
                fprintf('Sampling rate: %u I and Q samples per second. \n', obj.fs)
              case ancortek.sampOpt.half
                obj.nSamp = obj.nSamp_max * 1/2;
                fprintf('Sampling at %u I and Q samples per sweep, 2 bytes per sample. \n', obj.nSamp)
                fprintf('Sampling rate: %u I and Q samples per second. \n', obj.fs)
              case ancortek.sampOpt.one
                obj.nSamp = obj.nSamp_max;
                fprintf('Sampling at %u I and Q samples per sweep, 2 bytes per sample. \n', obj.nSamp)
                fprintf('Sampling rate: %u I and Q samples per second. \n', obj.fs)
            end
            
            obj.nSamp_up = floor(obj.nSamp * obj.Tramp_up / obj.sTime);
        end
        
        function setTx(obj,~,~)
        %setTx Set active transmitter

            obj.cmd_setTx = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setTx(1:obj.cmd_rep) = obj.setTx_h + obj.activeTx;
            ancortek.sdradar_send_data(obj.cmd_setTx);
            
            switch obj.activeTx
              case ancortek.txOpt.tx1 
                obj.nTx = 1;
                fprintf('Transmitter No. 1 is active. \n')
              case ancortek.txOpt.tx2
                obj.nTx = 1;
                fprintf('Transmitter No. 2 is active. \n')
              case ancortek.txOpt.tx12
                obj.nTx = 2;
                fprintf('Transmitter No. 1 and No. 2 are active. \n')
            end          
        end
        
        function setRx(obj,~,~)
        %setRx Set active receiver

            obj.cmd_setRx = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setRx(1:obj.cmd_rep) = obj.setRx_h + obj.activeRx;
            ancortek.sdradar_send_data(obj.cmd_setRx);
            switch obj.activeRx
              case ancortek.rxOpt.rx1
                obj.nRx = 1;
                fprintf('Receiver No. 1 is active. \n')
              case ancortek.rxOpt.rx2 
                obj.nRx = 1;
                fprintf('Receiver No. 2 is active. \n')
              case ancortek.rxOpt.rx3 
                obj.nRx = 1;
                fprintf('Receiver No. 3 is active. \n')
              case ancortek.rxOpt.rx4
                obj.nRx = 1;
                fprintf('Receiver No. 4 is active. \n')
              case ancortek.rxOpt.rx12
                obj.nRx = 2;
                fprintf('Receiver No. 1 and No. 2 are active. \n')
              case ancortek.rxOpt.rx34
                obj.nRx = 2;
                fprintf('Receiver No. 3 and No. 4 are active. \n')
              case ancortek.rxOpt.rx1234
                obj.nRx = 4;
                fprintf('Receiver No. 1, 2, 3 and 4 are active. \n');
            end
            
        % Update the number of samples per sweep becuase it depends on the number of receiver and sweep time.
            if ~isempty(obj.samplesPerSweep) % The 'samplesPerSweep' property has to be initialized in the 'init' method
                obj.nSamp_max = obj.max_data_rate*obj.sTime/obj.nRx;
                obj.nSamp = obj.nSamp_max * 2^(-double(obj.samplesPerSweep));
                obj.nSamp_up = floor(obj.nSamp * obj.Tramp_up/obj.sTime);
                fprintf('Sampling at %u I and Q samples per sweep, 2 bytes per sample. \n', obj.nSamp)
                fprintf('Sampling rate: %u I and Q samples per second. \n', obj.fs)
            end
        end
        
        function PLL_register_setup_sawtooth(obj)
            %PLL_REGISTER_SETUP_SAWTOOTH Set up PLL registers for FMCW
            %sawtooth
            %   Used for setting PLL regesters for FMCW bandwidth

            Tref  = 1/50e6;
            
            switch obj.sTime
                case 125e-6
                    Max_Sweepover = 1024;
                    T_downchirp = 5e-6;
                    Tramp_percent = 1-T_downchirp*1.5/obj.sTime;
                    Sweep_N = ceil(Max_Sweepover*(Tramp_percent+0.01));
                    obj.Tramp_up = obj.sTime*Tramp_percent;
                case 250e-6
                    Max_Sweepover = 2048;
                    T_downchirp = 5e-6;
                    Tramp_percent = 1-T_downchirp*1.5/obj.sTime;
                    Sweep_N = ceil(Max_Sweepover*(Tramp_percent+0.01));
                    obj.Tramp_up = obj.sTime*Tramp_percent;
                case 0.5e-3
                    Max_Sweepover = 4096;
                    T_downchirp = 70e-6;
                    Tramp_percent = 1-T_downchirp*1.5/obj.sTime;
                    Sweep_N = ceil(Max_Sweepover*(Tramp_percent+0.01));
                    obj.Tramp_up = obj.sTime*Tramp_percent;
                case 1e-3
                    Max_Sweepover = 8192;
                    T_downchirp = 90e-6;
                    Tramp_percent = 1-T_downchirp*1.5/obj.sTime;
                    Sweep_N = ceil(Max_Sweepover*(Tramp_percent+0.01));
                    obj.Tramp_up = obj.sTime*Tramp_percent;
                case 2e-3
                    Max_Sweepover = 16384;
                    T_downchirp = 110e-6;
                    Tramp_percent = 1-T_downchirp*1.5/obj.sTime;
                    Sweep_N = ceil(Max_Sweepover*(Tramp_percent+0.01));
                    obj.Tramp_up = obj.sTime*Tramp_percent;
                case 4e-3
                    Max_Sweepover = 32768;
                    T_downchirp = 90e-6;
                    Tramp_percent = 1-T_downchirp*1.5/obj.sTime;
                    Sweep_N = ceil(Max_Sweepover*(Tramp_percent+0.01));
                    obj.Tramp_up = obj.sTime*Tramp_percent;
                case 8e-3
                    Max_Sweepover = 65536;
                    T_downchirp = 90e-6;
                    Tramp_percent = 1-T_downchirp*1.5/obj.sTime;
                    Sweep_N = ceil(Max_Sweepover*(Tramp_percent+0.01));
                    obj.Tramp_up = obj.sTime*Tramp_percent;
            end
            
            % update nSamp_up
            obj.nSamp_up = floor(obj.nSamp * obj.Tramp_up / obj.sTime);
            
            if obj.fStart > 20e9
                % Frequency at prescalar: f_vco = f_ps/16
                F_start = obj.fStart  / 16;
                F_stop = obj.fStop  / 16;
            elseif (obj.fStart <= 20e9 && obj.fStart >= 3e9)
                % Frequency at prescalar: f_vco = f_ps/2
                F_start = obj.fStart / 2;
                F_stop = obj.fStop / 2;
            else
                F_start = obj.fStart;
                F_stop = obj.fStop;
            end
            
            % Calculate togglebutton_activate N and Stop N
            Start_N = F_start / 50e6;
            Stop_N = F_stop / 50e6;
            Start_N_int = floor(Start_N);
            Start_N_frac = Start_N - Start_N_int;
            % Stop_N_int = floor(Stop_N);
            % Stop_N_frac = Stop_N - Stop_N_int;
            Reg_03h = Start_N_int;
            Reg_04h = round(Start_N_frac * 2^24);
            
            % number of reference cycles in Tramp_up
            Nbr_of_Steps = obj.Tramp_up / Tref;
            
            % desired N step size, given togglebutton_activate N, Stop N and Nbr of steps
            N_Step_Size_desired = (Stop_N - Start_N) / Nbr_of_Steps;
            
            % Quantize the fractional N step into the 24 bit step size
            Frac_N_Step_Size = round(N_Step_Size_desired * 2^24);
            Reg_0Ah = Frac_N_Step_Size;
            
            % Readjust the stop frequency to ensure it falls exactly on a step boundary
            % Target an accurate stop frequency, at the expense of sweep time accuracy
            % Given step size of Frac_N_step_Size/2^24, how many cycles to get from
            % togglebutton_activate N to Stop N
            Nbr_of_Steps = round((Stop_N - Start_N)/(Frac_N_Step_Size/2^24));
            
            % Stop N in real
            % Stop_N_real = Start_N + Nbr_of_Steps * Frac_N_Step_Size / 2^24;
            
            % Number of big steps
            Nbr_of_Big_Steps = floor(Nbr_of_Steps*Frac_N_Step_Size/2^24);
            Reg_0Ch = Start_N_int + Nbr_of_Big_Steps;
            Reg_0Dh =  mod(Nbr_of_Steps*Frac_N_Step_Size, 2^24) + Reg_04h;
            
            if Reg_0Dh > 2^24
                Reg_0Ch = Reg_0Ch + 1;
                Reg_0Dh = Reg_0Dh - 2^24;
            end
            
            obj.setSweepstop_Hi = obj.setSweepstop_Hi_h + floor(Sweep_N/2^8);
            obj.setSweepstop_Lo = obj.setSweepstop_Lo_h + mod(Sweep_N, 2^8);
            
            obj.setReg03_Hi = obj.setReg03_Hi_h + floor(Reg_03h/2^16);
            tmp = mod(Reg_03h, 2^16);
            obj.setReg03_Mi = obj.setReg03_Mi_h + floor(tmp/2^8);
            obj.setReg03_Lo = obj.setReg03_Lo_h + mod(tmp, 2^8);
            
            obj.setReg04_Hi = obj.setReg04_Hi_h + floor(Reg_04h/2^16);
            tmp = mod(Reg_04h, 2^16);
            obj.setReg04_Mi = obj.setReg04_Mi_h + floor(tmp/2^8);
            obj.setReg04_Lo = obj.setReg04_Lo_h + mod(tmp, 2^8);
            
            obj.setReg0A_Hi = obj.setReg0A_Hi_h + floor(Reg_0Ah/2^16);
            tmp = mod(Reg_0Ah, 2^16);
            obj.setReg0A_Mi = obj.setReg0A_Mi_h + floor(tmp/2^8);
            obj.setReg0A_Lo = obj.setReg0A_Lo_h + mod(tmp, 2^8);
            
            obj.setReg0C_Hi = obj.setReg0C_Hi_h + floor(Reg_0Ch/2^16);
            tmp = mod(Reg_0Ch, 2^16);
            obj.setReg0C_Mi = obj.setReg0C_Mi_h + floor(tmp/2^8);
            obj.setReg0C_Lo = obj.setReg0C_Lo_h + mod(tmp, 2^8);
            
            obj.setReg0D_Hi = obj.setReg0D_Hi_h + floor(Reg_0Dh/2^16);
            tmp = mod(Reg_0Dh, 2^16);
            obj.setReg0D_Mi = obj.setReg0D_Mi_h + floor(tmp/2^8);
            obj.setReg0D_Lo = obj.setReg0D_Lo_h + mod(tmp, 2^8);
                      
            % NOTE: Combining all registers into one commnand doesn't work. We
            % have to set each register one by one.
            
            obj.cmd_setSweepstop_Hi = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setSweepstop_Hi(1:obj.cmd_rep) = obj.setSweepstop_Hi;
            
            obj.cmd_setSweepstop_Lo = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setSweepstop_Lo(1:obj.cmd_rep) = obj.setSweepstop_Lo;
            
            obj.cmd_setReg03_Hi = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg03_Hi(1:obj.cmd_rep) = obj.setReg03_Hi;
            
            obj.cmd_setReg03_Mi = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg03_Mi(1:obj.cmd_rep) = obj.setReg03_Mi;
            
            obj.cmd_setReg03_Lo = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg03_Lo(1:obj.cmd_rep) = obj.setReg03_Lo;
            
            obj.cmd_setReg04_Hi = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg04_Hi(1:obj.cmd_rep) = obj.setReg04_Hi;
            
            obj.cmd_setReg04_Mi = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg04_Mi(1:obj.cmd_rep) = obj.setReg04_Mi;
            
            obj.cmd_setReg04_Lo = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg04_Lo(1:obj.cmd_rep) = obj.setReg04_Lo;
            
            obj.cmd_setReg0A_Hi = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg0A_Hi(1:obj.cmd_rep) = obj.setReg0A_Hi;
            
            obj.cmd_setReg0A_Mi = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg0A_Mi(1:obj.cmd_rep) = obj.setReg0A_Mi;
            
            obj.cmd_setReg0A_Lo = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg0A_Lo(1:obj.cmd_rep) = obj.setReg0A_Lo;
            
            obj.cmd_setReg0C_Hi = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg0C_Hi(1:obj.cmd_rep) = obj.setReg0C_Hi;
            
            obj.cmd_setReg0C_Mi = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg0C_Mi(1:obj.cmd_rep) = obj.setReg0C_Mi;
            
            obj.cmd_setReg0C_Lo = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg0C_Lo(1:obj.cmd_rep) = obj.setReg0C_Lo;
            
            obj.cmd_setReg0D_Hi = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg0D_Hi(1:obj.cmd_rep) = obj.setReg0D_Hi;
            
            obj.cmd_setReg0D_Mi = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg0D_Mi(1:obj.cmd_rep) = obj.setReg0D_Mi;
            
            obj.cmd_setReg0D_Lo = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg0D_Lo(1:obj.cmd_rep) = obj.setReg0D_Lo;
            
            ancortek.sdradar_send_data(obj.cmd_setSweepstop_Hi);
            ancortek.sdradar_send_data(obj.cmd_setSweepstop_Lo);
            ancortek.sdradar_send_data(obj.cmd_setReg03_Hi);
            ancortek.sdradar_send_data(obj.cmd_setReg03_Mi);
            ancortek.sdradar_send_data(obj.cmd_setReg03_Lo);
            ancortek.sdradar_send_data(obj.cmd_setReg04_Hi);
            ancortek.sdradar_send_data(obj.cmd_setReg04_Mi);
            ancortek.sdradar_send_data(obj.cmd_setReg04_Lo);
            ancortek.sdradar_send_data(obj.cmd_setReg0A_Hi);
            ancortek.sdradar_send_data(obj.cmd_setReg0A_Mi);
            ancortek.sdradar_send_data(obj.cmd_setReg0A_Lo);
            ancortek.sdradar_send_data(obj.cmd_setReg0C_Hi);
            ancortek.sdradar_send_data(obj.cmd_setReg0C_Mi);
            ancortek.sdradar_send_data(obj.cmd_setReg0C_Lo);
            ancortek.sdradar_send_data(obj.cmd_setReg0D_Hi);
            ancortek.sdradar_send_data(obj.cmd_setReg0D_Mi);
            ancortek.sdradar_send_data(obj.cmd_setReg0D_Lo);
        end
        
        function PLL_register_setup_triangle(obj)
            %PLL_REGISTER_SETUP_TRIANGLE Set up PLL registers for FMCW
            %triangle
            %   Used for setting PLL regesters for FMCW bandwidth
            
            Tref  = 1/50e6;
            
            obj.Tramp_up = obj.sTime/2*0.9;
            obj.Tramp_dn = obj.Tramp_up;
            
            if obj.fStart > 20e9
                % Frequency at prescalar: f_vco = f_ps/16
                F_start = obj.fStart  / 16;
                F_stop = obj.fStop  / 16;
            elseif (obj.fStart <= 20e9 && obj.fStart >= 3e9)
                % Frequency at prescalar: f_vco = f_ps/2
                F_start = obj.fStart / 2;
                F_stop = obj.fStop / 2;
            else
                F_start = obj.fStart;
                F_stop = obj.fStop;
            end
            
            % Calculate togglebutton_activate N and Stop N
            Start_N = F_start / 50e6;
            Stop_N = F_stop / 50e6;
            Start_N_int = floor(Start_N);
            Start_N_frac = Start_N - Start_N_int;
            % Stop_N_int = floor(Stop_N);
            % Stop_N_frac = Stop_N - Stop_N_int;
            Reg_03h = Start_N_int;
            Reg_04h = round(Start_N_frac * 2^24);
            
            % number of reference cyles in Tramp_up
            Nbr_of_Steps = obj.Tramp_up / Tref;
            
            % desired N step size, given togglebutton_activate N, Stop N and Nbr of steps
            N_Step_Size_desired = (Stop_N - Start_N) / Nbr_of_Steps;
            
            % Quantize the fractional N step into the 24 bit step size
            Frac_N_Step_Size = round(N_Step_Size_desired * 2^24);
            Reg_0Ah = Frac_N_Step_Size;
            
            % Readjust the stop frequency to ensure it falls exactly on a step boundary
            % Target an accurate stop frequency, at the expense of sweep time accuracy
            % Given step size of Frac_N_step_Size/2^24, how many cycles to get from
            % togglebutton_activate N to Stop N
            Nbr_of_Steps = round((Stop_N - Start_N)/(Frac_N_Step_Size/2^24));
            
            % Stop N in real
            % Stop_N_real = Start_N + Nbr_of_Steps * Frac_N_Step_Size / 2^24;
            
            % Number of big stepsf
            Nbr_of_Big_Steps = floor(Nbr_of_Steps*Frac_N_Step_Size/2^24);
            Reg_0Ch = Start_N_int + Nbr_of_Big_Steps;
            Reg_0Dh =  mod(Nbr_of_Steps*Frac_N_Step_Size, 2^24) + Reg_04h;
            
            if Reg_0Dh > 2^24
                Reg_0Ch = Reg_0Ch + 1;
                Reg_0Dh = Reg_0Dh - 2^24;
            end
            
            obj.setReg03_Hi = obj.setReg03_Hi_h + floor(Reg_03h/2^16);
            tmp = mod(Reg_03h, 2^16);
            obj.setReg03_Mi = obj.setReg03_Mi_h + floor(tmp/2^8);
            obj.setReg03_Lo = obj.setReg03_Lo_h + mod(tmp, 2^8);
            
            obj.setReg04_Hi = obj.setReg04_Hi_h + floor(Reg_04h/2^16);
            tmp = mod(Reg_04h, 2^16);
            obj.setReg04_Mi = obj.setReg04_Mi_h + floor(tmp/2^8);
            obj.setReg04_Lo = obj.setReg04_Lo_h + mod(tmp, 2^8);
            
            obj.setReg0A_Hi = obj.setReg0A_Hi_h + floor(Reg_0Ah/2^16);
            tmp = mod(Reg_0Ah, 2^16);
            obj.setReg0A_Mi = obj.setReg0A_Mi_h + floor(tmp/2^8);
            obj.setReg0A_Lo = obj.setReg0A_Lo_h + mod(tmp, 2^8);
            
            obj.setReg0C_Hi = obj.setReg0C_Hi_h + floor(Reg_0Ch/2^16);
            tmp = mod(Reg_0Ch, 2^16);
            obj.setReg0C_Mi = obj.setReg0C_Mi_h + floor(tmp/2^8);
            obj.setReg0C_Lo = obj.setReg0C_Lo_h + mod(tmp, 2^8);
            
            obj.setReg0D_Hi = obj.setReg0D_Hi_h + floor(Reg_0Dh/2^16);
            tmp = mod(Reg_0Dh, 2^16);
            obj.setReg0D_Mi = obj.setReg0D_Mi_h + floor(tmp/2^8);
            obj.setReg0D_Lo = obj.setReg0D_Lo_h + mod(tmp, 2^8);
            
            % NOTE: Combining all registers into one commnand doesn't work. We
            % have to set each register one by one.
            
            obj.cmd_setReg03_Hi = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg03_Hi(1:obj.cmd_rep) = obj.setReg03_Hi;
            
            obj.cmd_setReg03_Mi = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg03_Mi(1:obj.cmd_rep) = obj.setReg03_Mi;
            
            obj.cmd_setReg03_Lo = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg03_Lo(1:obj.cmd_rep) = obj.setReg03_Lo;
            
            obj.cmd_setReg04_Hi = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg04_Hi(1:obj.cmd_rep) = obj.setReg04_Hi;
            
            obj.cmd_setReg04_Mi = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg04_Mi(1:obj.cmd_rep) = obj.setReg04_Mi;
            
            obj.cmd_setReg04_Lo = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg04_Lo(1:obj.cmd_rep) = obj.setReg04_Lo;
            
            obj.cmd_setReg0A_Hi = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg0A_Hi(1:obj.cmd_rep) = obj.setReg0A_Hi;
            
            obj.cmd_setReg0A_Mi = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg0A_Mi(1:obj.cmd_rep) = obj.setReg0A_Mi;
            
            obj.cmd_setReg0A_Lo = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg0A_Lo(1:obj.cmd_rep) = obj.setReg0A_Lo;
            
            obj.cmd_setReg0C_Hi = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg0C_Hi(1:obj.cmd_rep) = obj.setReg0C_Hi;
            
            obj.cmd_setReg0C_Mi = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg0C_Mi(1:obj.cmd_rep) = obj.setReg0C_Mi;
            
            obj.cmd_setReg0C_Lo = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg0C_Lo(1:obj.cmd_rep) = obj.setReg0C_Lo;
            
            obj.cmd_setReg0D_Hi = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg0D_Hi(1:obj.cmd_rep) = obj.setReg0D_Hi;
            
            obj.cmd_setReg0D_Mi = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg0D_Mi(1:obj.cmd_rep) = obj.setReg0D_Mi;
            
            obj.cmd_setReg0D_Lo = uint16(zeros(obj.cmd_len, 1));
            obj.cmd_setReg0D_Lo(1:obj.cmd_rep) = obj.setReg0D_Lo;
            
            ancortek.sdradar_send_data(obj.cmd_setSweepstop_Hi);
            ancortek.sdradar_send_data(obj.cmd_setSweepstop_Lo);
            ancortek.sdradar_send_data(obj.cmd_setReg03_Hi);
            ancortek.sdradar_send_data(obj.cmd_setReg03_Mi);
            ancortek.sdradar_send_data(obj.cmd_setReg03_Lo);
            ancortek.sdradar_send_data(obj.cmd_setReg04_Hi);
            ancortek.sdradar_send_data(obj.cmd_setReg04_Mi);
            ancortek.sdradar_send_data(obj.cmd_setReg04_Lo);
            ancortek.sdradar_send_data(obj.cmd_setReg0A_Hi);
            ancortek.sdradar_send_data(obj.cmd_setReg0A_Mi);
            ancortek.sdradar_send_data(obj.cmd_setReg0A_Lo);
            ancortek.sdradar_send_data(obj.cmd_setReg0C_Hi);
            ancortek.sdradar_send_data(obj.cmd_setReg0C_Mi);
            ancortek.sdradar_send_data(obj.cmd_setReg0C_Lo);
            ancortek.sdradar_send_data(obj.cmd_setReg0D_Hi);
            ancortek.sdradar_send_data(obj.cmd_setReg0D_Mi);
            ancortek.sdradar_send_data(obj.cmd_setReg0D_Lo);
        end
    end
end