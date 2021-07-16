# from pycaster.pycaster import rayCaster
# # import rayCaster
# import os
# import inspect
# import random
# # print(inspect.getfile(rayCaster))

# print(random.normalvariate(0, 0.2))

# # def solution(n, costs):

# #     costs = sorted(costs, key = lambda x : (x[2], x[1]))
# #     last = costs[0][0]
# #     answer = 0
# #     for cost in costs:
        
# #         if last + 1 == cost[0]:
# #             answer += cost[2] 
# #             last = cost[1]

# #     return answer

# # print(solution(4, [[0,1,1],[0,2,2],[1,2,5],[1,3,1],[2,3,8]]))


# import the required libraries 
import random 
import matplotlib.pyplot as plt 
    
# store the random numbers in a list 
nums = []
numsg = []
mu = 0
sigma = 50
    
for i in range(1000000): 
    temp = random.normalvariate(mu, sigma) 
    nums.append(temp)
    temp = random.gauss(mu, sigma) 
    numsg.append(temp)

fig, axes = plt.subplots(2,1)

plt.subplot(2,1,1)
plt.hist(nums, bins = 200) 
plt.title("normalvariate random")


plt.subplot(2,1,2)
plt.hist(numsg, bins = 200) 
plt.title("gauss random")

fig.tight_layout()

# plotting a graph 
# plt.hist(nums, bins = 200) 
plt.show()
