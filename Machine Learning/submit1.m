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

%% Project the Data
[U,S,V] = svd([X_train;X_test]);

dimension = 500;
project_x_train = (X_train-mean2(X_train))*V(:,1:dimension);
project_x_test = (X_test-mean2(X_test))*V(:,1:dimension);

upper = max(project_x_train);
lower = min(project_x_train);
N = size(project_x_train,1);

upperTest = max(project_x_test);
lowerTest = min(project_x_test);
NTest = size(project_x_test,1);

scaled_train = (project_x_train - repmat(lower,N,1))./repmat(upper-lower,N,1);
scaled_test = (project_x_test - repmat(lowerTest,NTest,1))./repmat(upperTest-lowerTest,NTest,1);
%% Prediction
y_label = double(y_train);
model = svmtrain(y_label,scaled_train,'-c 500 -g 0.07');

prediction1 = svmpredict(ones(15000,1),scaled_test,model);

%% Save submission
csvwrite('submission3.csv',[[1:15000]',prediction1]);

%% Show a few images

a = find(prediction1==6,40);
for i=1:40
    subplot(8,5,i);
    imshow(imresize(reshape(X_train(a(i),:),32,32,3),[256,256]));
end