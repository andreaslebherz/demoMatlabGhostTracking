function [spherical_detections, cartesian_detections] = helperConvertDetections(detections)

    input_type = detections{1}.MeasurementParameters.Frame;
    
    if input_type == 'Rectangular'
        print("Rectangular to spherical conversion");
    
    else
        print("Spherical to rectangular conversion");
    end
end