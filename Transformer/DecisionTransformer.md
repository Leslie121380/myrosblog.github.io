# 使用Transformer和RL构建序列检测模型用于动作预测（Offline Learning）
## 1. 基本原理

   🍀基本Transformer的Encoder-Decoder架构中，编码器所起到的关键作用就是把一个句子中每个词本身的意义，词的位置信息和词与词之间的关联性都编码到矩阵当中作为解码器的输入
   解码器对其进行解码并给出最优的回答预测，其优势是*全局意识*和*关联性*意识。
   
   🍀基本RL的Bellman方程中，`Q(s,a)=R(s,a)+γmaxQ'(s,a)`，通过不断与环境交互，使用TD误差对模型Q进行校准，关键参数是r(即R),s,a，代表状态s下执行动作a的即时反馈r=R(s,a)，
    该模型可以做到与环境交互，但是下一时刻的状态只会收到上一时刻的影响，当动作不断进行，模型存在遗忘的风险，因为它记不住之前的s。
    
🪢Combination!!!

   把原来Bellman方程中的这个Q(s,a)中的s换成被Transformer的编码模块进行Encode系列操作后的序列编码向量序列编码向量:
   
   <img width="1630" height="578" alt="image" src="https://github.com/user-attachments/assets/f403b42b-a965-4210-b993-78fd361e62d1" />

   这样过往的所有状态和反馈都被编码进序列，Bellman方程可以带着之前的"记忆”不断和环境交互学习，这也是offline learning（离线学习）的原理。就是用已有数据集去训练模型Q，训练完成之后再使用这个Q去应对类似或者新的环境，这个也是以下代码的训练逻辑。

## 2. 实现

step1: 生成训练数据集（使用cartpole）模型
```
def create_dataset():
    env = gym.make('CartPole-v1')
    dataset = []
    
    for episode in range(200): # 收集 200 场比赛
        states, actions, rewards, returns_to_go = [], [], [], []
        state, _ = env.reset()
        done = False
        episode_data = {'observations': [], 'actions': [], 'rewards': []}
        
        while not done:
            # 专家策略：简单的平衡逻辑
            action = 0 if state[2] < 0 else 1 
            episode_data['observations'].append(state)
            episode_data['actions'].append([action])
            
            state, reward, term, trunc, _ = env.step(action)
            episode_data['rewards'].append(reward)
            done = term or trunc
            
        # 计算 Return-to-go (从当前时刻到结束的总分)
        rews = episode_data['rewards']
        rtg = []
        total_rew = sum(rews)
        for i in range(len(rews)):
            rtg.append([total_rew])
            total_rew -= rews[i]
        episode_data['returns_to_go'] = rtg
        dataset.append(episode_data)

    with open('data/cartpole_expert.pkl', 'wb') as f:
        pickle.dump(dataset, f)
    print("数据集已生成至 data/ 文件夹！")

if __name__ == "__main__":
    create_dataset()
```
step2: 用刚才的架构思路搭建Decision Transformer训练架构

```
class DecisionTransformer(nn.Module):
    def __init__(self, state_dim, act_dim, hidden_size):
        super().__init__()
        # 嵌入层
        self.embed_rtg = nn.Linear(1, hidden_size)# Return-to-go 也是一个标量，映射到 hidden_size，是我们期望的输出
        self.embed_state = nn.Linear(state_dim, hidden_size)#构造s的嵌入层，输入维度是状态空间的维度，输出维度是隐藏层维度
        self.embed_action = nn.Embedding(act_dim, hidden_size)#构造a的嵌入层，输入维度是动作空间的维度，输出维度是隐藏层维度
        self.embed_timestep = nn.Embedding(1024, hidden_size)#构造时间步的嵌入层，假设最多1000步，相当于原来transformer中的位置编码，这里是用时间戳来对前后发生的动作进行区分

        # Transformer 层
        self.transformer = nn.TransformerEncoder(
            nn.TransformerEncoderLayer(d_model=hidden_size, nhead=4, batch_first=True),
            num_layers=3#用函数直接堆叠出一个三层的TransformerEncoder结构，这个玩意就是我之前写的那个transformer的Encoder部分，这里直接调用函数实现了
        )

        # 预测头：预测下一个动作
        self.predict_action = nn.Linear(hidden_size, act_dim)#最终的输出是一个动作概率分布，act_dim是动作空间的维度，告诉我下一步应该采取哪个动作

    def forward(self, states, actions, rtgs, timesteps):#用刚刚构造的嵌入层来嵌入输入的状态、动作、回报和时间步信息
        # 1. 映射到隐藏层维度
        s_emb = self.embed_state(states)
        a_emb = self.embed_action(actions.squeeze(-1))
        r_emb = self.embed_rtg(rtgs)
        t_emb = self.embed_timestep(timesteps)

        # 2. 加入时间步信息
        s_emb, a_emb, r_emb = s_emb + t_emb, a_emb + t_emb, r_emb + t_emb#将时间步向量 叠加 到状态、动作和奖励向量上。这样模型就知道这些数据分别属于哪个时刻

        # 3. 拼接序列 [R1, S1, A1, R2, S2, A2...]
        batch_size, seq_len, _ = states.shape
        # 交叉堆叠
        stacked_inputs = torch.stack((r_emb, s_emb, a_emb), dim=1).permute(0, 2, 1, 3).reshape(batch_size, 3*seq_len, -1)#把数据排成 [R1, S1, A1, R2, S2, A2...] 的格式
        
        # 4. 进入 Transformer
        outputs = self.transformer(stacked_inputs)#使用transformer中的encoder编码结构来处理序列，通过 Self-Attention，每一个位置都能“看到”之前发生的所有事情
        
        # 5. 只取 S 对应的输出位置预测动作 (索引 1, 4, 7...)
        action_preds = self.predict_action(outputs[:, 1::3, :])#取到所有的s，给预测函数预测下一步动作，输出一个动作概率分布
        return action_preds
```

step3: 把数据集喂给step2搭建好的`decision_trans`模型进行训练
```
# 参数设置
batch_size = 64
max_len = 20 # 观察过去20步
state_dim, act_dim, hidden_size = 4, 2, 128

with open('data/cartpole_expert.pkl', 'rb') as f:
    trajectories = pickle.load(f)

def get_batch():
    # 随机采样轨迹并切片的简易逻辑
    s, a, r, t = [], [], [], []
    for _ in range(batch_size):
        traj = trajectories[np.random.randint(len(trajectories))]
        start = np.random.randint(0, len(traj['observations']) - max_len)
        s.append(traj['observations'][start:start+max_len])
        a.append(traj['actions'][start:start+max_len])
        r.append(traj['returns_to_go'][start:start+max_len])
        t.append(np.arange(start, start+max_len))
    return torch.tensor(s, dtype=torch.float32), torch.tensor(a, dtype=torch.long), \
           torch.tensor(r, dtype=torch.float32), torch.tensor(t, dtype=torch.long)

model = DecisionTransformer(state_dim, act_dim, hidden_size)
optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)
loss_fn = torch.nn.CrossEntropyLoss()

for epoch in range(2000):
    states, actions, rtgs, timesteps = get_batch()
    preds = model(states, actions, rtgs, timesteps)
    
    # 损失：预测动作 vs 专家动作
    loss = loss_fn(preds.reshape(-1, act_dim), actions.reshape(-1))
    
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()
    
    if epoch % 100 == 0:
        print(f"Epoch {epoch}, Loss: {loss.item():.4f}")

torch.save(model.state_dict(), "dt_model.pt")
```
step4: 模型训练完成之后进入新的cartpole环境使用
```
env = gym.make('CartPole-v1', render_mode="human")
model = DecisionTransformer(4, 2, 128)
model.load_state_dict(torch.load("dt_model.pt"))
model.eval()

# 1. 初始环境重置
state, _ = env.reset()
target_return = 500.0
max_len = 20 # 必须与训练时的长度一致

# 2. 初始化全零缓冲区 (Batch_size=1, Seq_len=20)
states = torch.zeros((1, max_len, 4), dtype=torch.float32)
actions = torch.zeros((1, max_len, 1), dtype=torch.long)
rtgs = torch.zeros((1, max_len, 1), dtype=torch.float32)
timesteps = torch.zeros((1, max_len), dtype=torch.long)

# 3. 游戏循环
for t in range(500):
    # --- 关键：将当前时刻的信息放入缓冲区的最后一位 ---
    states[0, -1] = torch.from_numpy(state).float()
    rtgs[0, -1] = torch.tensor([target_return]).float()
    timesteps[0, -1] = min(t, 1023) # 避免超过 Embedding 范围

    with torch.no_grad():
        # 此时 states, actions, rtgs 全都是 [1, 20, d]，维度绝对相等
        preds = model(states, actions, rtgs, timesteps)
        # 我们只取序列最后一个位置的预测结果
        action = torch.argmax(preds[0, -1]).item()

    # 4. 执行动作
    state, reward, term, trunc, _ = env.step(action)
    target_return -= reward # 模拟目标递减
    
    # --- 关键：滑动窗口，为下一帧腾出位置 ---
    # 所有张量向左滚动 1 位，原来的 [-1] 跑到 [-2]，新的 [-1] 变回 0 等待下次填充
    states = torch.roll(states, shifts=-1, dims=1)
    actions = torch.roll(actions, shifts=-1, dims=1)
    rtgs = torch.roll(rtgs, shifts=-1, dims=1)
    timesteps = torch.roll(timesteps, shifts=-1, dims=1)
    
    # 记录刚才执行的动作（放在 -2 的位置，因为 -1 位置将被下一轮的 state 覆盖前清空）
    actions[0, -2] = action 

    if term or trunc:
        # t 是从 0 开始的索引，所以坚持的总时长是 t + 1
        print(f">>> 任务结束 | 坚持时长: {t + 1} 帧 | 剩余目标回报: {target_return:.2f}")
        break
env.close()
```




   
   




   
