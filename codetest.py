def solution(n, costs):

    costs = sorted(costs, key = lambda x : (x[2], x[1]))
    last = costs[0][0]
    answer = 0
    for cost in costs:
        
        if last + 1 == cost[0]:
            answer += cost[2] 
            last = cost[1]

    return answer

print(solution(4, [[0,1,1],[0,2,2],[1,2,5],[1,3,1],[2,3,8]]))