%% Classification
% 1 = Plane
% 2 = Car
% 3 = Bird
% 4 = Cat
% 5 = Game
% 6 = Dog
% 7 = Frog
% 8 = Horse
% 9 = Boat
% 10 = Truck

%% Fit Naive bayes

NB = fitNaiveBayes(X_train,y_label);

prediction2 = predict(NB,X_test);

%% Save submission
csvwrite('submission4.csv',[[1:15000]',prediction2]);

%% Show a few images

a = find(prediction2==4,40);
for i=1:40
    subplot(8,5,i);
    imshow(imresize(reshape(X_train(a(i),:),32,32,3),[256,256]));
end