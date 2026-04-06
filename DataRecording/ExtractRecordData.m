
load PUP20230806T192612
whos
Tx1Rx1=ComplexDataTx1(1:256, :);
Tx1Rx2=ComplexDataTx1(257:512, :);
figure(1)
plot(abs(Tx1Rx1(:, 16)))
figure(2)
plot(abs(Tx1Rx2(:, 200)))