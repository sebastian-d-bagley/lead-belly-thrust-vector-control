function thrust = Motor_Simulation(time)
% Returns thrust in newtons for given time in seconds by interpolating the
% engine thrust curve
    persistent t_data f_data initialized;

    % Load the engine data
    if isempty(initialized)
        engine_data = {
            '<eng-data cg="49." f="0." m="38.8" t="0."/>';
            '<eng-data cg="49." f="57.631" m="38.1838" t="0.039"/>';
            '<eng-data cg="49." f="53.491" m="33.6753" t="0.187"/>';
            '<eng-data cg="49." f="51.239" m="29.2252" t="0.342"/>';
            '<eng-data cg="49." f="47.86" m="24.9328" t="0.5"/>';
            '<eng-data cg="49." f="33.806" m="13.7388" t="1."/>';
            '<eng-data cg="49." f="22.94" m="5.96068" t="1.5"/>';
            '<eng-data cg="49." f="10.135" m="1.42709" t="2."/>';
            '<eng-data cg="49." f="4.504" m="0.596372" t="2.207"/>';
            '<eng-data cg="49." f="0." m="0." t="2.69"/>';
        };

        N = numel(engine_data);
        t_raw = zeros(N, 1);
        f_raw = zeros(N, 1);

        % Get the time and thrust values
        for k = 1:N
            s = engine_data{k};
            
            % Find and extract time
            t_pattern_start = 't="';
            idx_t_pattern_start = strfind(s, t_pattern_start);
            idx_t_value_start = idx_t_pattern_start(1) + length(t_pattern_start);
            idx_t_value_end_relative = strfind(s(idx_t_value_start:end), '"');
            idx_t_value_end = idx_t_value_start + idx_t_value_end_relative(1) - 2;
            t_str = s(idx_t_value_start : idx_t_value_end);
            t_raw(k) = real(str2double(t_str));

            % Find and extract force
            f_pattern_start = 'f="';
            idx_f_pattern_start = strfind(s, f_pattern_start);
            idx_f_value_start = idx_f_pattern_start(1) + length(f_pattern_start);
            idx_f_value_end_relative = strfind(s(idx_f_value_start:end), '"');
            idx_f_value_end = idx_f_value_start + idx_f_value_end_relative(1) - 2;
            f_str = s(idx_f_value_start : idx_f_value_end);
            f_raw(k) = real(str2double(f_str));
        end

        % Sort data by time
        [t_sorted, sort_idx] = sort(t_raw);
        f_sorted = f_raw(sort_idx);

        % handle any duplicates
        [t_unique, unique_idx] = unique(t_sorted, 'last');
        
        % store everything in the persistent variable
        t_data = t_unique;
        f_data = f_sorted(unique_idx);

        initialized = true;
    end

    % get the thrust each step from initialization
    thrust = interp1(t_data, f_data, time, 'linear', 0);

end
