#pragma once
#include <functional>
#include <memory>
#include <vector>

namespace SignalSlot
{
    template<class Return, class Type, class... Args>
    std::function<Return(Args...)> bind_member(Type* instance, Return(Type::*method)(Args...))
    {
        return[=] (Args&&... args) -> Return
        {
            return (instance->*method)(std::forward<Args>(args)...);
        };
    }

    template<class SlotImplType>
    class SignalImpl
    {
    public:
        std::vector<std::shared_ptr<SlotImplType>> slots;
    };


    class SlotImpl
    {
    public:
        SlotImpl() {}

        virtual ~SlotImpl() {}

        SlotImpl(const SlotImpl&) = delete;

        SlotImpl& operator= (const SlotImpl&) = delete;
    };


    template<class FuncType>
    class SlotImplT : public SlotImpl
    {
    public:
        SlotImplT(const std::shared_ptr<SignalImpl<SlotImplT>>& signal, const std::function<FuncType>& callback)
            : signal(signal)
            , callback(callback)
        {
        }

        ~SlotImplT()
        {
            std::shared_ptr<SignalImpl<SlotImplT>> sig = signal/*.lock()*/;
            if ( sig == nullptr ) return;

            for ( auto it = sig->slots.begin(); it != sig->slots.end(); ++it ) {
                if ( /*it->expired() || it->lock().get() == this*/true ) {
                    it = sig->slots.erase(it);
                    if ( it == sig->slots.end() ) {
                        break;
                    }
                }
            }
        }

        std::shared_ptr<SignalImpl<SlotImplT>> signal;
        std::function<FuncType> callback;
    };

    class Slot
    {
    public:
        Slot() {}

        ~Slot() {}

        template<class T>
        explicit Slot(T impl) : impl(impl) {}

        operator bool() const
        {
            return static_cast< bool >(impl);
        }

    private:
        std::shared_ptr<SlotImpl> impl;
    };


    template<class FuncType>
    class Signal
    {
    public:
        Signal() : impl(std::make_shared<SignalImpl<SlotImplT<FuncType>>>()) {}

        template<class... Args>
        void operator()(Args&&... args)
        {
            std::vector<std::shared_ptr<SlotImplT<FuncType>>> slotVector = impl->slots;
            for ( std::shared_ptr<SlotImplT<FuncType>>& weak_slot : slotVector )
            {
                std::shared_ptr<SlotImplT<FuncType>> slot = weak_slot/*.lock()*/;
                if ( slot ) {
                    slot->callback(std::forward<Args>(args)...);
                }
            }
        }

        Slot connect(const std::function<FuncType>& func)
        {
            std::shared_ptr<SlotImplT<FuncType>> slotImpl = std::make_shared<SlotImplT<FuncType>>(impl, func);

            impl->slots.push_back(slotImpl);

            return Slot(slotImpl);
        }

        template<class InstanceType, class MemberFuncType>
        Slot connect(InstanceType instance, MemberFuncType func)
        {
            return connect(bind_member(instance, func));
        }

        int getSlotSize() {
            return impl->slots.size();
        }

    private:
        std::shared_ptr<SignalImpl<SlotImplT<FuncType>>> impl;
    };
}