function q_result = quaternionMultiply(q1, q2)
    w1 = q1(1); x1 = q1(2); y1 = q1(3); z1 = q1(4);
    w2 = q2(1); x2 = q2(2); y2 = q2(3); z2 = q2(4);
    
    scalarPart = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
    vectorPart = [w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
                  w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
                  w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2];
    
    q_result = [scalarPart; vectorPart];
end