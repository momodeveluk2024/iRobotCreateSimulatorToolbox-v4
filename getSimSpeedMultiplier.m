function speedMultiplier = getSimSpeedMultiplier(handlesGUI)
% getSimSpeedMultiplier Returns the selected simulator speed from the GUI.
speedMultiplier = 1;

popupSpeed = [];
if isfield(handlesGUI, 'popup_speed') && ishandle(handlesGUI.popup_speed)
    popupSpeed = handlesGUI.popup_speed;
elseif isfield(handlesGUI, 'figure_simulator') && ishandle(handlesGUI.figure_simulator)
    popupSpeed = findall(handlesGUI.figure_simulator, 'Tag', 'popup_speed');
end

if ~isempty(popupSpeed) && ishandle(popupSpeed)
    speedMultiplier = round(get(popupSpeed(1), 'Value'));
    speedMultiplier = min(max(speedMultiplier, 1), 10);
    return
end

if isfield(handlesGUI, 'radio_speed2') && ishandle(handlesGUI.radio_speed2) && ...
        get(handlesGUI.radio_speed2, 'Value')
    speedMultiplier = 2;
elseif isfield(handlesGUI, 'radio_speed3') && ishandle(handlesGUI.radio_speed3) && ...
        get(handlesGUI.radio_speed3, 'Value')
    speedMultiplier = 3;
end
end
