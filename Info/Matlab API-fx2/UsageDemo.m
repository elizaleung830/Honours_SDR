% Matlab API usage demo

%% Initialization
sdr = ancortek.sdradar;
sdr.init;
 
%% Setting parameters 
sdr.modulation = ancortek.modOpt.FMCW_sawtooth;
sdr.activeRx = ancortek.rxOpt.rx1;
sdr.activeTx = ancortek.txOpt.tx1;
sdr.sweeptime = ancortek.stOpt.us1000;
sdr.samplesPerSweep = ancortek.sampOpt.one;
sdr.bw_update(24e9,25.5e9)
% sdr.bw_update(5.6e9,6.0e9)

%% Set up plots
time_scope = figure('name', 'Time Scope', 'unit', 'normalized','outerposition',[0 0 0.5 0.5]);
axis_I = subplot(2,1,1);
title('I')
axis_Q = subplot(2,1,2);
title('Q')
axis_I.XLabel.String = 'Sampling number';
axis_I.YLabel.String = 'Magnitude';
axis_I.YLim = [-2000 2000];
axis_Q.XLabel.String = 'Sampling number';
axis_Q.YLabel.String = 'Magnitude';
axis_Q.YLim = [-2000 2000];

hLines_I = cell(sdr.nRx*sdr.nTx, 1);
hLines_Q = cell(sdr.nRx*sdr.nTx, 1);
colors = {'r','g','b','k','m','c','y',[0.5 0.5 0.5]};

switch sdr.modulation
    case ancortek.modOpt.FMCW_sawtooth
        for ii = 1:(sdr.nRx*sdr.nTx)
            hLines_I{ii} = line(axis_I, 1:sdr.nSamp_up, NaN(1, sdr.nSamp_up), 'DisplayName', sprintf('Rx %d', ii), 'Color', colors{ii});
            hLines_Q{ii} = line(axis_Q, 1:sdr.nSamp_up, NaN(1, sdr.nSamp_up), 'DisplayName', sprintf('Rx %d', ii), 'Color', colors{ii});
        end
    case ancortek.modOpt.CW
        for ii = 1:(sdr.nRx*sdr.nTx)
            hLines_I{ii} = line(axis_I, 1:sdr.nSamp, NaN(1, sdr.nSamp), 'DisplayName', sprintf('Rx %d', ii), 'Color', colors{ii});
            hLines_Q{ii} = line(axis_Q, 1:sdr.nSamp, NaN(1, sdr.nSamp), 'DisplayName', sprintf('Rx %d', ii), 'Color', colors{ii});
        end
    case ancortek.modOpt.FMCW_triangle
        for ii = 1:(sdr.nRx*sdr.nTx)
            hLines_I{ii} = line(axis_I, 1:sdr.nSamp, NaN(1, sdr.nSamp), 'DisplayName', sprintf('Rx %d', ii), 'Color', colors{ii});
            hLines_Q{ii} = line(axis_Q, 1:sdr.nSamp, NaN(1, sdr.nSamp), 'DisplayName', sprintf('Rx %d', ii), 'Color', colors{ii});
        end               
end

legend(axis_I)
legend(axis_Q)

%%
while 1    
    try
        [Is, Qs] = sdr.get_IQCube(2); % fast samples by receving channels by slow samples
        
        if sdr.nTx == 2
            % virtual array
           Is_t1 = Is(:,:,1:2:end);
           Is_t2 = Is(:,:,2:2:end);
           Qs_t1 = Qs(:,:,1:2:end);
           Qs_t2 = Qs(:,:,2:2:end);
           Is = cat(2, Is_t1, Is_t2);
           Qs = cat(2, Qs_t1, Qs_t2);
        end
    catch
        continue
    end        
    
   if ~isempty(Is) && ~isempty(Qs)
       % plot Is/Qs from all receivers on one plot
       for ii=1:(sdr.nTx*sdr.nRx)
           hLines_I{ii}.YData = Is(:, ii, 2);
           hLines_Q{ii}.YData = Qs(:, ii, 2);
       end
       
       drawnow
   end
    
   if ishandle(time_scope) 
        if (get(time_scope,'currentkey')=='q')
            close(time_scope);            
            break
        end
    end
end
