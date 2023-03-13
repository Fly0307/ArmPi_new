class Order:
    def __init__(self, orderID, rect, box):
        self.orderID = orderID
        self.rect = rect
        self.box = box

    def __eq__(self, other):
        return self.orderID == other.orderID

    def __hash__(self):
        return hash(self.orderID)

# # 插入一个订单
# new_order = Order("001", (0, 0), (10, 10))
# orders.add(new_order)

# # 删除一个订单
# existing_order = Order("001", (0, 0), (10, 10))
# orders.remove(existing_order)

# # 更新一个订单
# existing_order = Order("001", (0, 0), (10, 10))
# orders.remove(existing_order)
# existing_order.rect = (0, 0)
# orders.add(existing_order)