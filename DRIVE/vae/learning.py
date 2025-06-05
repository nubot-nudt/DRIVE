""" Learning utilities """
from functools import partial
from torch.optim import Optimizer

class EarlyStopping(object): # pylint: disable=R0902
    """
    Gives a criterion to stop training when a given metric is not
    improving anymore
    Args:
        mode (str): One of `min`, `max`. In `min` mode, training will
            be stopped when the quantity monitored has stopped
            decreasing; in `max` mode it will be stopped when the
            quantity monitored has stopped increasing. Default: 'min'.
        patience (int): Number of epochs with no improvement after
            which training is stopped. For example, if
            `patience = 2`, then we will ignore the first 2 epochs
            with no improvement, and will only stop learning after the
            3rd epoch if the loss still hasn't improved then.
            Default: 10.
        threshold (float): Threshold for measuring the new optimum,
            to only focus on significant changes. Default: 1e-4.
        threshold_mode (str): One of `rel`, `abs`. In `rel` mode,
            dynamic_threshold = best * ( 1 + threshold ) in 'max'
            mode or best * ( 1 - threshold ) in `min` mode.
            In `abs` mode, dynamic_threshold = best + threshold in
            `max` mode or best - threshold in `min` mode. Default: 'rel'.

    """

    def __init__(self, mode='min', patience=10, threshold=1e-4, threshold_mode='rel'):
        self.patience = patience
        self.mode = mode
        self.threshold = threshold
        self.threshold_mode = threshold_mode
        self.best = None
        self.num_bad_epochs = None
        self.mode_worse = None  # the worse value for the chosen mode
        self.is_better = None
        self.last_epoch = -1
        self._init_is_better(mode=mode, threshold=threshold,
                             threshold_mode=threshold_mode)
        self._reset()

    def _reset(self):
        """Resets num_bad_epochs counter and cooldown counter."""
        self.best = self.mode_worse
        self.num_bad_epochs = 0

    def step(self, metrics, epoch=None):
        """ Updates early stopping state """
        current = metrics
        if epoch is None:
            epoch = self.last_epoch = self.last_epoch + 1
        self.last_epoch = epoch

        if self.is_better(current, self.best):
            self.best = current
            self.num_bad_epochs = 0
        else:
            self.num_bad_epochs += 1

    @property
    def stop(self):
        """ Should we stop learning? """
        return self.num_bad_epochs > self.patience


    def _cmp(self, mode, threshold_mode, threshold, a, best): # pylint: disable=R0913, R0201
        if mode == 'min' and threshold_mode == 'rel':
            rel_epsilon = 1. - threshold
            return a < best * rel_epsilon

        elif mode == 'min' and threshold_mode == 'abs':
            return a < best - threshold

        elif mode == 'max' and threshold_mode == 'rel':
            rel_epsilon = threshold + 1.
            return a > best * rel_epsilon

        return a > best + threshold

    def _init_is_better(self, mode, threshold, threshold_mode):
        if mode not in {'min', 'max'}:
            raise ValueError('mode ' + mode + ' is unknown!')
        if threshold_mode not in {'rel', 'abs'}:
            raise ValueError('threshold mode ' + threshold_mode + ' is unknown!')

        if mode == 'min':
            self.mode_worse = float('inf')
        else:  # mode == 'max':
            self.mode_worse = (-float('inf'))

        self.is_better = partial(self._cmp, mode, threshold_mode, threshold)

    def state_dict(self):
        """ Returns early stopping state """
        return {key: value for key, value in self.__dict__.items() if key != 'is_better'}

    def load_state_dict(self, state_dict):
        """ Loads early stopping state """
        self.__dict__.update(state_dict)
        self._init_is_better(mode=self.mode, threshold=self.threshold,
                             threshold_mode=self.threshold_mode)



############################################################
#### WARNING : THIS IS A TEMPORARY CODE WHICH HAS      #####
####  TO BE REMOVED WITH PYTORCH 0.5                   #####
#### IT IS COPY OF THE 0.5 VERSION OF THE LR SCHEDULER #####
############################################################
class ReduceLROnPlateau(object): # pylint: disable=R0902
    """
    这段代码实现了一个自定义的 ReduceLROnPlateau 类，用于在训练过程中根据某个监控指标（如验证集损失）的变化动态调整学习率。
    它的主要作用是当监控指标在一定数量的训练周期（patience）内没有改善时，自动降低学习率。
    这种策略可以帮助模型在训练过程中避免陷入局部最优，从而提高模型的性能。

    以下是代码的主要功能和作用的总结：
    1. 动态调整学习率

        当监控指标（如验证集损失）在 patience 个训练周期内没有改善时，自动降低学习率。
        学习率的调整方式为：new_lr = lr * factor，其中 factor 是一个小于 1 的因子（默认为 0.1）。

    2. 监控指标的模式

        支持两种模式：
            min：当监控指标停止下降时，降低学习率。
            max：当监控指标停止上升时，降低学习率。

    3. 阈值和阈值模式

        threshold：用于判断监控指标是否有所改善的阈值（默认为 1e-4）。
        threshold_mode：阈值的模式，可以是：
            rel：相对阈值，动态阈值为 best * (1 + threshold)（max 模式）或 best * (1 - threshold)（min 模式）。
            abs：绝对阈值，动态阈值为 best + threshold（max 模式）或 best - threshold（min 模式）。

    4. 冷却期

        cooldown：在学习率降低后，设置一个冷却期（默认为 0），在此期间忽略监控指标的变化。
        冷却期结束后，恢复正常的监控和学习率调整。

    5. 最小学习率

        min_lr：学习率的下限，可以是一个标量或一个列表，分别对应每个参数组的最小学习率。

    6. 其他功能

        verbose：如果设置为 True，会在调整学习率时打印信息。
        eps：最小衰减量，如果新旧学习率的差值小于 eps，则忽略更新。

    7. 方法

        step(metrics, epoch=None)：在每个训练周期结束时调用，更新调度器状态。
        _reduce_lr(epoch)：实际降低学习率的方法。
        state_dict() 和 load_state_dict(state_dict)：用于保存和加载调度器的状态。

    8. 使用示例

        optimizer = torch.optim.SGD(model.parameters(), lr=0.1, momentum=0.9)
        scheduler = ReduceLROnPlateau(optimizer, mode='min', factor=0.1, patience=10, verbose=True)

        for epoch in range(10):
            train(...)
            val_loss = validate(...)
            scheduler.step(val_loss)

    9. 主要作用

        动态调整学习率：当监控指标在一定周期内没有改善时，自动降低学习率。
        避免过早停止：通过设置 patience 和 cooldown，避免因短期波动导致过早调整学习率。
        提高模型性能：通过动态调整学习率，帮助模型更好地收敛，避免陷入局部最优。

    这个类的功能与 PyTorch 的 torch.optim.lr_scheduler.ReduceLROnPlateau 类似，但提供了更多的自定义选项，例如自定义阈值模式和冷却期。
    """

    def __init__(self, optimizer, mode='min', factor=0.1, patience=10, # pylint: disable=R0913
                 verbose=False, threshold=1e-4, threshold_mode='rel',
                 cooldown=0, min_lr=0, eps=1e-8):

        if factor >= 1.0:
            raise ValueError('Factor should be < 1.0.')
        self.factor = factor # 0.5

        if not isinstance(optimizer, Optimizer):
            raise TypeError('{} is not an Optimizer'.format(
                type(optimizer).__name__))
        self.optimizer = optimizer

        if isinstance(min_lr, (list, tuple)):
            if len(min_lr) != len(optimizer.param_groups):
                raise ValueError("expected {} min_lrs, got {}".format(
                    len(optimizer.param_groups), len(min_lr)))
            self.min_lrs = list(min_lr)
        else:
            self.min_lrs = [min_lr] * len(optimizer.param_groups)

        self.patience = patience # 10
        self.verbose = verbose # True
        self.cooldown = cooldown # 0
        self.cooldown_counter = 0
        self.mode = mode # 'min'
        self.threshold = threshold # 1e-4
        self.threshold_mode = threshold_mode # 'rel'
        self.best = None
        self.num_bad_epochs = None
        self.mode_worse = None  # the worse value for the chosen mode
        self.is_better = None
        self.eps = eps # 1e-8
        self.last_epoch = -1
        self._init_is_better(mode=mode, threshold=threshold,
                             threshold_mode=threshold_mode)
        self._reset()

    def _reset(self):
        """Resets num_bad_epochs counter and cooldown counter."""
        self.best = self.mode_worse
        self.cooldown_counter = 0
        self.num_bad_epochs = 0

    def step(self, metrics, epoch=None):
        """ Updates scheduler state """
        current = metrics
        if epoch is None:
            epoch = self.last_epoch = self.last_epoch + 1
        self.last_epoch = epoch

        if self.is_better(current, self.best):
            self.best = current
            self.num_bad_epochs = 0
        else:
            self.num_bad_epochs += 1

        if self.in_cooldown:
            self.cooldown_counter -= 1
            self.num_bad_epochs = 0  # ignore any bad epochs in cooldown

        if self.num_bad_epochs > self.patience:
            self._reduce_lr(epoch)
            self.cooldown_counter = self.cooldown
            self.num_bad_epochs = 0

    def _reduce_lr(self, epoch):
        for i, param_group in enumerate(self.optimizer.param_groups):
            old_lr = float(param_group['lr'])
            new_lr = max(old_lr * self.factor, self.min_lrs[i])
            if old_lr - new_lr > self.eps:
                param_group['lr'] = new_lr
                if self.verbose:
                    print('Epoch {:5d}: reducing learning rate'
                          ' of group {} to {:.4e}.'.format(epoch, i, new_lr))

    @property
    def in_cooldown(self):
        """ Are we on CD? """
        return self.cooldown_counter > 0

    def _cmp(self, mode, threshold_mode, threshold, a, best): # pylint: disable=R0913,R0201
        if mode == 'min' and threshold_mode == 'rel':
            rel_epsilon = 1. - threshold
            return a < best * rel_epsilon

        elif mode == 'min' and threshold_mode == 'abs':
            return a < best - threshold

        elif mode == 'max' and threshold_mode == 'rel':
            rel_epsilon = threshold + 1.
            return a > best * rel_epsilon

        return a > best + threshold

    def _init_is_better(self, mode, threshold, threshold_mode):
        if mode not in {'min', 'max'}:
            raise ValueError('mode ' + mode + ' is unknown!')
        if threshold_mode not in {'rel', 'abs'}:
            raise ValueError('threshold mode ' + threshold_mode + ' is unknown!')

        if mode == 'min':
            self.mode_worse = float('inf')
        else:  # mode == 'max':
            self.mode_worse = (-float('inf'))

        self.is_better = partial(self._cmp, mode, threshold_mode, threshold)

    def state_dict(self):
        """ Returns scheduler state """
        return {key: value for key, value in self.__dict__.items()
                if key not in {'optimizer', 'is_better'}}

    def load_state_dict(self, state_dict):
        """ Loads scheduler state """
        self.__dict__.update(state_dict)
        self._init_is_better(mode=self.mode, threshold=self.threshold,
                             threshold_mode=self.threshold_mode)